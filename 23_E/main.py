from maix import camera, display, image, nn, app, uart, pinmap, time, touchscreen
from serial_protocol import SerialProtocol
from blob_detect import BlobDetector
from threshold import ColorThresholdConfig
from menu import MenuInterface
from servo import ServoController
from pid import PIDIncrementalController

SCREEN_WIDTH, SCREEN_HEIGHT = 320, 240
CAMERA_RESOLUTION = (SCREEN_WIDTH, SCREEN_HEIGHT)

class DisplayManager:
    _instance = None
    @classmethod
    def get_instance(cls):
        if cls._instance is None:
            cls._instance = display.Display()
        return cls._instance

start_flag = 0

# 根据色块颜色选择对应配置
red_thresholds = [[0, 80, 40, 80, 10, 80]]      # red
# green_thresholds = [[0, 80, -120, -10, 0, 30]]    # green
blue_thresholds = [[18, 68, 0, 67, -92, -22]]  # blue
# 创建色块检测器实例
red_detector = BlobDetector(red_thresholds, 50)
blue_detector = BlobDetector(blue_thresholds, 50)
red_x, red_y = 0, 0
blue_x, blue_y = 0, 0
red_flag, blue_flag = 0, 0

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

servo_flag = False
servo_180 = ServoController(180)
servo_270 = ServoController(270)
ctrl_angle_180 = 90
ctrl_angle_270 = 135

pid_x = PIDIncrementalController(0.08, 0.035, 0.1, 100)
# pid_x = PIDIncrementalController(0, 0, 0, 100)
pid_x.limit(120)
pid_x.set_point(CAMERA_RESOLUTION[0] // 2)
pid_y = PIDIncrementalController(0.1, 0.03, 0, 100)
pid_y.limit(90)
pid_y.set_point(CAMERA_RESOLUTION[1] // 2)

while not app.need_exit():
    img = cam.read()
    red_result = red_detector.detect_max_blob(img)
    blue_result = blue_detector.detect_max_blob(img)
    if red_result:
        red_x, red_y = red_result[0]
        # red_x, red_y = red_detector.sliding_filter(red_x, red_y)
        img.draw_cross(red_x, red_y, image.COLOR_RED, 5, 2)
    if blue_result:
        blue_x, blue_y = blue_result[0]
        # blue_x, blue_y = blue_detector.sliding_filter(blue_x, blue_y)
        img.draw_cross(blue_x, blue_y, image.COLOR_BLUE, 5, 2)

    if servo_flag:
        pid_x.update(blue_x)
        ctrl_angle_270 = ctrl_angle_270 - pid_x.output
        pid_y.update(blue_y)
        ctrl_angle_180 = ctrl_angle_180 - pid_y.output

        servo_180.set_angle(ctrl_angle_180)
        servo_270.set_angle(ctrl_angle_270)
        print(blue_x, blue_y, ctrl_angle_270, ctrl_angle_180)

    menu.render(img)
    menu.update()
    red_flag, blue_flag, start_flag = menu.get_flags()
    if red_flag:
        config = ColorThresholdConfig(cam, disp, ts, red_thresholds) # 运行阈值调整
        red_thresholds = config.run_threshold_adjust()
        red_detector.set_threshold([red_thresholds])
    if blue_flag:
        config = ColorThresholdConfig(cam, disp, ts, blue_thresholds) # 运行阈值调整
        blue_thresholds = config.run_threshold_adjust()
        blue_detector.set_threshold([blue_thresholds])
    if start_flag:
        servo_flag = True
        

    # # SEND
    # payload = struct.pack('<iiii', red_x, red_y, blue_x, blue_y)
    # encoded = com_proto.encode(payload)
    # serial.write(encoded)
        
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