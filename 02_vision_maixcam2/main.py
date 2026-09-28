from maix import app, camera, display, image, nn, time, uart, touchscreen

from ball_position import (
    AdaptiveAlphaBetaFilter,
    axis_point,
    position_from_pixel,
    validate_calibration,
)
from hybrid_tracker import HybridBallTracker


# ============================== 功能开关 ==============================

DEBUG_LOG = False
DRAW_DEBUG = False
SHOW_FPS = False
USE_WEBRTC = True
PRINT_PROTOCOL = False
LENS_CORR_ENABLE = True
LENS_CORR_STRENGTH = 0.6
TOUCH_ENABLE = True
TOUCH_DRAW = True
TOUCH_SEND_EDGE_ONLY = False

LOW_LATENCY_MODE = True


# ============================== 模型与标定 ==============================

MODEL_PATH = "models/yolo26_all_maixcam2_yolo26_640_160/yolo26_all.mud"

AXIS_START_PX = (67, 80)
AXIS_END_PX = (392, 81)
AXIS_START_CM = 2.5
AXIS_END_CM = 15.5


detector = nn.YOLO26(
    model=MODEL_PATH,
    dual_buff=not LOW_LATENCY_MODE,
)

FRAME_WIDTH = detector.input_width()
FRAME_HEIGHT = detector.input_height()


# ============================== 串口协议 ==============================


def debug_print(message):
    if DEBUG_LOG:
        print(message)


def send_ball_result(valid, position_cm, velocity_pixel_s, quality, now_ms, touch_cm=None):
    if valid:
        payload = "$BALL,1,{:.3f},{:.1f},{:.3f},{}".format(
            position_cm, velocity_pixel_s, quality, now_ms
        )
    else:
        payload = "$BALL,0,0,0,0,{}".format(now_ms)
    if touch_cm is not None:
        payload += ",{:.3f}".format(touch_cm)
    payload += "*\n"

    serial.write_str(payload)

    if PRINT_PROTOCOL:
        print(payload, end="")


# ============================== 绘制 ==============================


def screen_to_image(touch_x, touch_y):
    scale = min(
        disp.width() / FRAME_WIDTH, disp.height() / FRAME_HEIGHT
    )
    offset_x = (disp.width() - FRAME_WIDTH * scale) / 2.0
    offset_y = (disp.height() - FRAME_HEIGHT * scale) / 2.0
    return (touch_x - offset_x) / scale, (touch_y - offset_y) / scale


def draw_installation_axis(frame):
    axis_color = image.Color.from_rgb(0, 220, 255)
    frame.draw_line(
        AXIS_START_PX[0],
        AXIS_START_PX[1],
        AXIS_END_PX[0],
        AXIS_END_PX[1],
        axis_color,
        2,
    )
    frame.draw_cross(AXIS_START_PX[0], AXIS_START_PX[1], axis_color, 7, 2)
    frame.draw_cross(AXIS_END_PX[0], AXIS_END_PX[1], axis_color, 7, 2)


# ============================== 初始化 ==============================

validate_calibration(
    AXIS_START_PX,
    AXIS_END_PX,
    AXIS_START_CM,
    AXIS_END_CM,
    FRAME_WIDTH,
    FRAME_HEIGHT,
)

cam = camera.Camera(FRAME_WIDTH, FRAME_HEIGHT, detector.input_format())
disp = display.Display()
serial = uart.UART("/dev/ttyS4", 115200)
ts = touchscreen.TouchScreen()
last_touch_pressed = False
tracker = HybridBallTracker(detector, FRAME_WIDTH, FRAME_HEIGHT)
position_filter = AdaptiveAlphaBetaFilter()

webrtc_server = None
if USE_WEBRTC:
    from maix import webrtc

    stream_channel = cam.add_channel(640, 180, image.Format.FMT_YVU420SP)
    webrtc_server = webrtc.WebRTC()
    webrtc_server.bind_camera(stream_channel)
    webrtc_server.start()
    print("WebRTC:", webrtc_server.get_url())

print(
    "Hybrid detector v1.0: {}x{}, model={}".format(
        FRAME_WIDTH, FRAME_HEIGHT, MODEL_PATH
    )
)

panel_color = image.Color.from_rgb(0, 0, 0)
projection_color = image.Color.from_rgb(255, 220, 0)
touch_color = image.Color.from_rgb(255, 80, 255)
source_colors = {
    "Y": image.COLOR_GREEN,
    "T": image.Color.from_rgb(255, 220, 0),
    "E": image.Color.from_rgb(0, 220, 255),
    "G": image.Color.from_rgb(255, 140, 0),
}
last_loop_ms = time.ticks_ms()


# ============================== 主循环 ==============================

while not app.need_exit():
    loop_start_ms = time.ticks_ms()
    loop_cost_ms = loop_start_ms - last_loop_ms
    last_loop_ms = loop_start_ms

    frame = cam.read()
    if LENS_CORR_ENABLE:
        frame = frame.lens_corr(strength=LENS_CORR_STRENGTH)

    detect_start_ms = time.ticks_ms()
    now_ms = time.ticks_ms()
    ball, debug = tracker.process(frame, now_ms)
    detect_cost_ms = time.ticks_ms() - detect_start_ms

    draw_installation_axis(frame)
    frame.draw_rect(0, 0, FRAME_WIDTH, 34, panel_color, -1)

    # ---------------------- 触摸输入 ----------------------
    touch_cm_for_send = None
    if TOUCH_ENABLE:
        touch_x, touch_y, touch_pressed = ts.read()
        send_touch = touch_pressed and (
            not TOUCH_SEND_EDGE_ONLY or not last_touch_pressed
        )
        last_touch_pressed = touch_pressed
        if send_touch:
            img_x, img_y = screen_to_image(touch_x, touch_y)
            if 0 <= img_x < FRAME_WIDTH and 0 <= img_y < FRAME_HEIGHT:
                touch_cm_for_send, _, _ = position_from_pixel(
                    (img_x, img_y),
                    AXIS_START_PX,
                    AXIS_END_PX,
                    AXIS_START_CM,
                    AXIS_END_CM,
                )
                if TOUCH_DRAW:
                    frame.draw_line(
                        int(round(img_x)), 0,
                        int(round(img_x)), FRAME_HEIGHT - 1,
                        touch_color, 2,
                    )
                    touch_text = "TOUCH {:+.2f}cm".format(touch_cm_for_send)
                    text_w = image.string_size(
                        touch_text, scale=1.0, thickness=2
                    ).width()
                    frame.draw_string(
                        max(0, FRAME_WIDTH - text_w - 5),
                        6,
                        touch_text,
                        color=touch_color,
                        scale=1.0,
                        thickness=2,
                    )

    # ---------------------- 识别结果 ----------------------
    if ball is not None:
        filtered_x, filtered_y = position_filter.update(
            ball["cx"], ball["cy"], now_ms
        )
        position_cm, axis_ratio, _ = position_from_pixel(
            (filtered_x, filtered_y),
            AXIS_START_PX,
            AXIS_END_PX,
            AXIS_START_CM,
            AXIS_END_CM,
        )
        projected_x, projected_y = axis_point(
            axis_ratio, AXIS_START_PX, AXIS_END_PX
        )

        source = ball["source"]
        box_color = source_colors.get(source, image.COLOR_GREEN)
        draw_x = max(0, int(ball["x"]))
        draw_y = max(0, int(ball["y"]))
        draw_w = max(1, min(FRAME_WIDTH - draw_x, int(ball["w"])))
        draw_h = max(1, min(FRAME_HEIGHT - draw_y, int(ball["h"])))
        frame.draw_rect(draw_x, draw_y, draw_w, draw_h, box_color, 2)
        frame.draw_cross(
            int(round(filtered_x)), int(round(filtered_y)), box_color, 9, 2
        )
        frame.draw_cross(
            int(round(projected_x)),
            int(round(projected_y)),
            projection_color,
            6,
            2,
        )

        frame.draw_string(
            8,
            5,
            "BALL {:+.2f}cm {}".format(position_cm, source),
            color=image.COLOR_WHITE,
            scale=1.25,
            thickness=2,
        )

        if DRAW_DEBUG:
            debug_text = "{} Q{:.2f} Y{} T{:.2f} {}ms".format(
                source,
                ball["score"],
                debug["raw_objects"],
                debug["template_score"],
                detect_cost_ms,
            )
            text_y = min(FRAME_HEIGHT - 18, draw_y + draw_h + 2)
            frame.draw_string(
                max(0, min(draw_x, FRAME_WIDTH - 205)),
                text_y,
                debug_text,
                color=box_color,
                scale=0.8,
                thickness=1,
            )

        send_ball_result(
            True,
            position_cm,
            position_filter.vx,
            ball["score"],
            now_ms,
            touch_cm_for_send,
        )
    else:
        position_filter.mark_missing(now_ms)
        frame.draw_string(
            8,
            5,
            "BALL LOST",
            color=image.COLOR_RED,
            scale=1.25,
            thickness=2,
        )
        if DRAW_DEBUG:
            debug_text = "Y{} E{} T{:.2f} G{:.2f} {}ms".format(
                debug["raw_objects"],
                debug["enhanced_objects"],
                debug["template_score"],
                debug["global_template_score"],
                detect_cost_ms,
            )
            frame.draw_string(
                8,
                38,
                debug_text,
                color=image.Color.from_rgb(255, 170, 0),
                scale=0.8,
                thickness=1,
            )
        send_ball_result(False, 0.0, 0.0, 0.0, now_ms, touch_cm_for_send)

    # ---------------------- 帧率显示 ----------------------
    if SHOW_FPS:
        fps = 0 if loop_cost_ms <= 0 else int(1000 / loop_cost_ms)
        fps_text = "FPS:{}".format(fps)
        text_width = image.string_size(
            fps_text, scale=1.0, thickness=1
        ).width()
        frame.draw_string(
            FRAME_WIDTH - text_width - 5,
            6,
            fps_text,
            color=image.COLOR_GREEN,
            scale=1.0,
            thickness=1,
        )

    debug_print(
        "source={} detect={}ms loop={}ms".format(
            debug["source"], detect_cost_ms, loop_cost_ms
        )
    )
    disp.show(frame)
