from maix import camera, display, image, nn, app, uart, pinmap, time, touchscreen
from serial_protocol import SerialProtocol
from blob_detect import BlobDetector
from threshold import ColorThresholdConfig
from menu import MenuInterface
from servo import ServoController
from pid import PIDIncrementalController
from black_rect_detector import BlackRectangleDetector
import struct

SCREEN_WIDTH, SCREEN_HEIGHT = 320, 240
CAMERA_RESOLUTION = (SCREEN_WIDTH, SCREEN_HEIGHT)

class DisplayManager:
    _instance = None
    @classmethod
    def get_instance(cls):
        if cls._instance is None:
            cls._instance = display.Display()
        return cls._instance

# ports = uart.list_devices()
# print(ports)
pinmap.set_pin_function("A16", "UART0_TX")
pinmap.set_pin_function("A17", "UART0_RX")
device = "/dev/ttyS0"
serial = uart.UART(device, 115200)
data_buffer = bytearray()
com_proto = SerialProtocol()

cam = camera.Camera(CAMERA_RESOLUTION[0], CAMERA_RESOLUTION[1])
# 修改所有display初始化处
# disp = display.Display()
disp = DisplayManager.get_instance()

ts = touchscreen.TouchScreen()
menu = MenuInterface(disp, ts, cam)

black_threshold = [[0, 10, -4, 7, -10, 20]]
rect_detector = BlackRectangleDetector(cam, black_threshold)
rect_x, rect_y = 0, 0
black_flag = False

black_detector = BlobDetector(black_threshold, 50)
# black_x, black_y = 0, 0

start_flag = 0
servo_flag = False
servo_180 = ServoController(180)
servo_270 = ServoController(270)
ctrl_angle_180 = 90
ctrl_angle_270 = 135

# pid_x = PIDIncrementalController(0.08, 0.035, 0.1, 100)
# pid_x.limit(120)
# pid_x.set_point(CAMERA_RESOLUTION[0] // 2)
# pid_y = PIDIncrementalController(0.1, 0.03, 0, 100)
# pid_y.limit(90)
# pid_y.set_point(CAMERA_RESOLUTION[1] // 2)

while not app.need_exit():
    img = cam.read()
    black_result = black_detector.detect_max_blob(img)
    if black_result is not None:
        if black_result[0]:
            black_x, black_y = black_result[0]
            img.draw_cross(black_x, black_y, image.COLOR_BLACK, 5, 2)
        max_blob = black_result[2]
        if max_blob:
            x, y, w, h = max_blob.rect()
            # 绘制矩形（左上角x, 左上角y, 右下角x, 右下角y, 颜色, 线宽）
            img.draw_rect(x, y, w, h, image.COLOR_RED, 2)
        

    # 获取矩形中心点
    rect_result = rect_detector.process_frame()
    if rect_result is not None:
        corners, center = rect_result
        if corners and len(corners) == 4:
    # 按顺序连接4个点，最后一个点连接回第一个点
            for i in range(4):
                x1, y1 = corners[i]
                x2, y2 = corners[(i+1) % 4]  # 取模运算实现循环连接
                img.draw_line(x1, y1, x2, y2, color=image.COLOR_BLUE, thickness=2)
            # 绘制十字交叉
            img.draw_cross(center[0], center[1], image.COLOR_GREEN, 5, 2)

    # if servo_flag:
    #     pid_x.update(blue_x)
    #     ctrl_angle_270 = ctrl_angle_270 - pid_x.output
    #     pid_y.update(blue_y)
    #     ctrl_angle_180 = ctrl_angle_180 - pid_y.output

    #     servo_180.set_angle(ctrl_angle_180)
    #     servo_270.set_angle(ctrl_angle_270)
    #     print(blue_x, blue_y, ctrl_angle_270, ctrl_angle_180)

    menu.render(img)
    menu.update()
    black_flag, start_flag = menu.get_flags()
    if black_flag:
        config = ColorThresholdConfig(cam, disp, ts, black_threshold) # 运行阈值调整
        black_threshold = config.run_threshold_adjust()
        black_detector.set_threshold(black_threshold)
    if start_flag:
        servo_flag = True
        

    # # SEND
    # payload = struct.pack('<iiii', red_x, red_y, blue_x, blue_y)
    # encoded = com_proto.encode(payload)
    # serial.write(encoded)
    # print(red_x,red_y,blue_x,blue_y)
        
    # # RECV
    # length = serial.available()
    # if length > 0:
    #     data = serial.read(length)
    #     data_buffer += data
    #     rc, bytes_redundant = com_proto.is_valid(data_buffer)
    #     if bytes_redundant > 0:
    #         data_buffer = data_buffer[bytes_redundant:]
    #     if rc >= 0:
    #         result = com_proto.decode(data_buffer)
    #         if len(result) == 16:
    #             x0,y0,x1,y1 = struct.unpack('<iiii', result)
    #             print('{},{},{},{}'.format(x0,y0,x1,y1))

    #     packet_length = com_proto.length(data_buffer)
    #     data_buffer = data_buffer[packet_length:]

    
    disp.show(img)