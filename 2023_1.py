# laser_control_system.py - MaixCam专用版本
from maix import image, camera, display
import time, math, pyb
import numpy as np

# ===== 初始化硬件 =====
# MaixCam摄像头初始化
cam = camera.Camera(320, 240)
cam.skip_frames(30)  # 跳过初始30帧
disp = display.Display()

# 其他硬件初始化
clock = time.clock()
uart = pyb.UART(3, 9600, timeout_char=10, read_buf_len=128)
led = pyb.LED(1)

# 全局变量
track_path = []
MODE = "None"
current_target = 0
cmd = "None"
last_spot = (0, 0)

# 颜色阈值配置 (Maix格式: [L, A, B])
RED_THRESHOLD = [0, 80, 40, 80, 10, 80]  # 红色激光点
BLACK_THRESHOLD = [0, 40, -20, 20, -20, 20]  # 黑色标记

# 校准参数
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
    blobs = img.find_blobs([RED_THRESHOLD], pixels_threshold=200)

    if blobs:
        # 选择最大色块
        blob = max(blobs, key=lambda b: b[2]*b[3])
        # 绘制十字标记
        img.draw_rect(blob[0], blob[1], blob[2], blob[3], image.COLOR_RED)
        img.draw_cross(blob[0]+blob[2]//2, blob[1]+blob[3]//2, image.COLOR_RED)
        return (blob[0]+blob[2]//2, blob[1]+blob[3]//2)
    return None

def save_current_point():
    """保存当前激光点位置"""
    global border_points, origin_point

    img = cam.read()
    spot = find_red_spot(img)

    if not spot:
        print("SPOT_NOT_FOUND")
        return False

    x, y = spot

    if calibration_mode == "ORIGIN":
        origin_point = (x, y)
        print("SAVE_ORIGIN", (x, y))
        return True

    elif calibration_mode == "BORDER":
        if len(border_points) < 4:
            border_points.append((x, y))
            print("SAVE_BORDER", border_points)
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
    return int(adjust_x), int(adjust_y)

def send_servo_command(data):
    data_bytes = bytes(data)
    packet = b'\xFF' + data_bytes + b'\xFE'
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
    # 限制角度范围
    return max(-127, min(angle, 127))

def hypot(x, y):
    """计算二维欧几里得距离"""
    if not isinstance(x, (int, float)) or not isinstance(y, (int, float)):
        raise TypeError("坐标值必须是数字")
    return (x**2 + y**2) **0.5

def detect_black_blocks(img):
    """检测黑色块"""
    blobs = img.find_blobs([BLACK_THRESHOLD], pixels_threshold=500)
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
    points = [(blob[0]+blob[2]//2, blob[1]+blob[3]//2) for blob in blobs]

    # 按顺时针排序
    sorted_points = sort_points_clockwise(points)

    # 添加第一个点使路径闭合
    sorted_points.append(sorted_points[0])

    return sorted_points

# PID控制器初始化
pid_x = PIDController(kp=0.3, ki=0, kd=0, limit=2)
pid_y = PIDController(kp=0.3, ki=0, kd=0, limit=2)

# 主循环
while True:
    img = cam.read()
    if not img:
        continue

    # 绘制边界点
    if len(border_points) >= 1:
        for (x, y) in border_points:
            img.draw_cross(x, y, image.COLOR_RED)

    # 绘制原点
    if origin_point:
        img.draw_cross(origin_point[0], origin_point[1], image.COLOR_GREEN)

    # 检测红色激光点
    spot = find_red_spot(img)
    if spot:
        last_spot = spot

    # === 指令处理 ===
    if uart.any():
        cmd = read_uart_packet()
        print("CMD:", cmd)

    if cmd == "SAVE_ORIGIN":
        calibration_mode = "ORIGIN"
        save_current_point()
        calibration_mode = "IDLE"
        cmd = "None"

    elif cmd == "SAVE_BORDER":
        calibration_mode = "BORDER"
        save_current_point()
        calibration_mode = "IDLE"
        cmd = "None"

    elif cmd == "START_BORDER":
        copy_current_target = []
        MODE = "BORDER"
        path = generate_path(border_points)
        cmd = "None"
        current_target = 0

    elif cmd == "START_RESET":
        direction = True  # True：正方向
        copy_reset_target = []
        current_origin = [last_spot, origin_point]
        path_origin = generate_path(current_origin)
        MODE = "RESET"
        cmd = "None"
        current_target_reset = 0

    elif cmd == "START_CLOSED_TRACK":
        MODE = "CLOSED_TRACK"
        cmd = "None"

    # === 模式执行 ===
    if MODE == "BORDER":
        if copy_current_target:
            for (x, y) in copy_current_target:
                img.draw_circle(int(x), int(y), 4, image.COLOR_RED)

        if path and current_target < len(path):
            # 绘制路径点
            for (x, y) in path:
                img.draw_cross(int(x), int(y), image.COLOR_BLUE)

            # 获取当前目标点
            target_x, target_y = path[current_target]
            angle_x, angle_y = calculate_servo_angles(
                target_x, target_y, last_spot[0], last_spot[1])

            # 发送舵机指令
            data = (clamp_angle(angle_x) + 127, clamp_angle(angle_y) + 127)
            send_servo_command(data)

            # 检查是否到达目标点
            if hypot(target_x - last_spot[0], target_y - last_spot[1]) < 6:
                copy_current_target.append(path[current_target])
                current_target += 1
                print(f"当前目标点: {current_target}/{len(path)}")

    elif MODE == "RESET":
        if copy_reset_target:
            for (x, y) in copy_reset_target:
                img.draw_circle(int(x), int(y), 4, image.COLOR_YELLOW)

        if path_origin and current_target_reset < len(path_origin):
            # 绘制复位路径
            for (x, y) in path_origin:
                img.draw_cross(int(x), int(y), image.COLOR_YELLOW)

            # 获取当前目标点
            target_x, target_y = path_origin[current_target_reset]
            angle_x, angle_y = calculate_servo_angles(
                target_x, target_y, last_spot[0], last_spot[1])

            # 发送舵机指令
            data = (clamp_angle(angle_x) + 127, clamp_angle(angle_y) + 127)
            send_servo_command(data)

            # 检查是否到达目标点
            if hypot(target_x - last_spot[0], target_y - last_spot[1]) < 6:
                copy_reset_target.append(path_origin[current_target_reset])
                current_target_reset += 1
                if current_target_reset >= len(path_origin):
                    MODE = "BORDER"
                    print("复位完成")

    elif MODE == "CLOSED_TRACK":
        track_path = generate_closed_path(img)
        if track_path:
            # 绘制闭合路径
            for i in range(len(track_path)-1):
                x1, y1 = track_path[i]
                x2, y2 = track_path[i+1]
                img.draw_line(x1, y1, x2, y2, image.COLOR_GREEN)

    # 显示图像
    disp.show(img)
    clock.tick()
