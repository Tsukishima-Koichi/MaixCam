# laser_control_system.py
import sensor, image, time, math, pyb


# ===== 初始化硬件 =====
sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)  # 320x240
sensor.set_auto_gain(False)
sensor.set_auto_whitebal(False)
sensor.skip_frames(time=500)
clock = time.clock() # Tracks F
uart = pyb.UART(3, 9600,timeout_char=10, read_buf_len=128)  # TX=P4, RX=P5
led = pyb.LED(1)

track_path = []

MODE="None"
current_target = 0

cmd="None"

RED_THRESHOLD =(100, 100, -1, 105, -17, 43)    # 红色激光点
BLACK_THRESHOLD = (0, 40, -20, 20, -20, 20) # 黑色标记
black_threshold=[0,60]

border_points = []  # 保存的边线点坐标
origin_point = None  # 原点坐标
calibration_mode = "IDLE"

class PIDController:
    def __init__(self, kp, ki, kd, limit):
        self.Kp = kp
        self.Ki = ki
        self.Kd = kd
        self.limit = limit
        self.reset()

    def reset(self):
        self.integral = 0
        self.last_error = 0
        self.last_time = time.ticks_ms()

    def update(self, error):
        now = time.ticks_ms()
        dt = time.ticks_diff(now, self.last_time) / 1000.0
        if dt <= 0: dt = 0.01

        # 计算微分项
        derivative = (error - self.last_error) / dt

        # 计算积分项（抗饱和）
        self.integral += error * dt
        if abs(self.integral) > self.limit:
            self.integral = math.copysign(self.limit, self.integral)

        # PID计算
        output = (self.Kp * error +
                 self.Ki * self.integral +
                 self.Kd * derivative)

        # 输出限幅
        output = max(-self.limit, min(self.limit, output))

        # 更新状态
        self.last_error = error
        self.last_time = now

        return output

def read_uart_packet(timeout=100):
    """读取完整数据包（自动拼接碎片数据）"""
    data = bytearray()
    start_time = time.ticks_ms()

    while time.ticks_diff(time.ticks_ms(), start_time) < timeout:
        chunk = uart.read()  # 读取可用数据
        if chunk:
            data.extend(chunk)

            # 简单协议：以'$'开头 '#'结尾
            if data.startswith(b'$') and b'#' in data:
                start_idx = data.find(b'$')
                end_idx = data.find(b'#')
                if start_idx < end_idx:
                    packet = data[start_idx+1:end_idx]
                    data = data[end_idx+1:]  # 保留未处理数据
                    return packet.decode()

    return None  # 超时返回None

def find_red_spot(img):
    """识别红色激光点位置"""
    blobs = img.find_blobs([RED_THRESHOLD])

    if blobs:
        blob=max(blobs, key=lambda b: b.area())
        img.draw_cross(blob.cx(), blob.cy(),color=(0, 0, 255))
        return blob
    return None

def save_current_point():
    """保存当前激光点位置"""
    global border_points, origin_point

    img = sensor.snapshot()
    spot = find_red_spot(img)

    if not spot:
        print("SPOT_NOT_FOUND")
        return False

    x, y = spot.cx(), spot.cy()

    if calibration_mode == "ORIGIN":
        origin_point = (x, y)
        print("SAVE_ORIGIN",(x, y))
        return True

    elif calibration_mode == "BORDER":
        if len(border_points) < 4:
            border_points.append((x, y))
            print("SAVE_BORDER",border_points)
            return True
        else:
            print("BORDER_FULL\n")
            return False
def calculate_servo_angles(target_x, target_y, current_x, current_y):
    """通过PID计算舵机角度"""
    # 计算误差（像素坐标）
    error_x = target_x - current_x
    error_y = target_y - current_y

    # PID更新
    adjust_x = pid_x.update(error_x)
    adjust_y = pid_y.update(error_y)

    # 转换为舵机角度

    # angle_x = max(0, min(SERVO_RANGE, angle_x))
    # angle_y = max(0, min(SERVO_RANGE, angle_y))

    return int(adjust_x), int(adjust_y)

def send_servo_command(data):

    data_bytes=bytes(data)
    packet=b'\xFF'+data_bytes+b'\xFE'
    uart.write(packet)

def generate_path(points, segments=20):
    """生成平滑路径点"""
    path = []
    for i in range(len(points)):
        start = points[i]
        end = points[(i+1)%len(points)]
        for j in range(segments):
            ratio = j / segments
            px = start[0] + (end[0]-start[0]) * ratio
            py = start[1] + (end[1]-start[1]) * ratio
            path.append((px, py))
    return path

def clamp_angle(angle):
    # 一步限制范围
    return max(-127, min(angle, 127))
def hypot(x, y):
    """计算二维欧几里得距离，带类型检查"""
    if not isinstance(x, (int, float)) or not isinstance(y, (int, float)):
        raise TypeError("坐标值必须是数字")
    return (x**2 + y**2) ** 0.5

def detect_black_blocks(img):
    """检测所有符合条件的黑色块"""
    # 转换为灰度图提高性能
    # gray = img.to_grayscale()
    # white_img = gray.binary([black_threshold])
    # 查找黑色块
    blobs = img.find_blobs([BLACK_THRESHOLD],
                           pixels_threshold=500,
                           # area_threshold=50,
                           merge=True)
    for blob in blobs:
       # 绘制红色矩形框
        img.draw_rectangle(blob.rect(), color=(255, 0, 0))
    # 过滤太小或太大的块
    # valid_blobs = []
    # for blob in blobs:
    #     if 100 <= blob.area() <= 10000:  # 避免过大块
    #         valid_blobs.append(blob)

    return blobs

def sort_points_clockwise(points):
    """将点按顺时针顺序排序"""
    if not points:
        return []

    # 计算中心点
    center_x = sum(p[0] for p in points) / len(points)
    center_y = sum(p[1] for p in points) / len(points)

    # 计算每个点的角度
    def angle_from_center(point):
        dx = point[0] - center_x
        dy = point[1] - center_y
        return math.atan2(dy, dx)  # 返回弧度值

    # 按角度排序（顺时针）
    sorted_points = sorted(points, key=angle_from_center, reverse=True)

    return sorted_points

def generate_closed_path(img):
    """生成闭合路径（顺时针）"""
    blobs = detect_black_blocks(img)
    if len(blobs) < 3:  # 至少需要3个点形成闭合路径
        return []

    # 获取中心点
    points = [(blob.cx(), blob.cy()) for blob in blobs]

    # 按顺时针排序
    sorted_points = sort_points_clockwise(points)

    # 添加第一个点使路径闭合
    sorted_points.append(sorted_points[0])

    return sorted_points


pid_x = PIDController(kp=0.3, ki=0, kd=0, limit=2)
pid_y = PIDController(kp=0.3, ki=0, kd=0, limit=2)

while True:
    img = sensor.snapshot()
    if len(border_points) >=1:
        for border_point in border_points:
            img.draw_cross(border_point[0], border_point[1],color=(0, 0, 255))

    if origin_point:
        img.draw_cross(origin_point[0], origin_point[1],color=(0, 0, 255))

    spot =find_red_spot(img)
    if spot:
                last_spot = (spot.cx(), spot.cy())
    clock.tick()

    # === 指令处理 ===
    if uart.any():
        cmd = read_uart_packet()
        print("CMD:", cmd)

    if cmd == "SAVE_ORIGIN":
        calibration_mode = "ORIGIN"
        save_current_point()
        calibration_mode = "IDLE"
        cmd="None"

    elif cmd == "SAVE_BORDER":
        calibration_mode = "BORDER"
        save_current_point()
        calibration_mode = "IDLE"
        cmd="None"# 不自动退出模式，可连续保存4个点

    elif cmd == "START_BORDER":
        copy_current_target=[]
        MODE = "BORDER"
        path = generate_path(border_points)
        cmd="None"
        current_target = 0

    elif cmd == "START_RESET":
        drection=not False #Ture：正方向
        copy_reset_target=[]
        current_origin = (last_spot,origin_point)
        path_origin = generate_path(current_origin)
        MODE = "RESET"
        cmd="None"
        current_target_reset = 0

    elif cmd == "START_CLOSED_TRACK":
        MODE = "CLOSED_TRACK"
        cmd="None"
        # current_target = 0
        # pid_x.reset()
        # pid_y.reset()



    if MODE == "BORDER":

        if copy_current_target:
            for copy in copy_current_target:
                img.draw_circle(int(copy[0]), int(copy[1]),4,color=(255, 0, 0))

        if path:
            for p in path:
                img.draw_cross(int(p[0]), int(p[1]),color=(0, 0, 255))


        if current_target < len(path):
            target_x, target_y = path[current_target]
            angle_x, angle_y = calculate_servo_angles(
                target_x, target_y, last_spot[0], last_spot[1])
            print(angle_x, angle_y)
            data = (clamp_angle(angle_x) + 127, clamp_angle(angle_y) + 127)
            send_servo_command(data)

            # 检查是否到达目标点
            if hypot(target_x-last_spot[0], target_y-last_spot[1]) < 6:

                copy_current_target.append(path[current_target])
                current_target = current_target + 1

                print(current_target)
    elif MODE == "RESET":
        if copy_reset_target:
            for copy in copy_reset_target:
                img.draw_circle(int(copy[0]), int(copy[1]),4,color=(255, 0, 0))

        if path_origin:
            for p in path_origin:
                img.draw_cross(int(p[0]), int(p[1]),color=(0, 0, 255))

        if current_target_reset < len(path_origin):
            target_x, target_y = path_origin[current_target_reset]
            angle_x, angle_y = calculate_servo_angles(
                target_x, target_y, last_spot[0], last_spot[1])

            print(angle_x, angle_y)
            data = (clamp_angle(angle_x) + 127, clamp_angle(angle_y) + 127)

            send_servo_command(data)

            if hypot(target_x-last_spot[0], target_y-last_spot[1]) < 6:

                copy_reset_target.append(path_origin[current_target_reset])
                current_target_reset = current_target_reset + 1
                print(current_target_reset)
        else:
            MODE = "BORDER"

    elif MODE == "CLOSED_TRACK":
        track_path=generate_closed_path(img)
        # 可视化调试
        if track_path:
            # 绘制黑色块中心点
            for point in track_path:
                img.draw_circle(point[0], point[1], 3, color=(0, 255, 0))
