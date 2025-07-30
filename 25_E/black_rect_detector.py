from maix import image, display, camera, app
from blob_detect import BlobDetector
import math

class BlackRectangleDetector:
    def __init__(self, cam, black_threshold):
        # 初始化摄像头
        self.cam = cam
        # 黑色阈值 (HSV颜色空间)
        self.black_threshold = black_threshold
        self.blob_detector = BlobDetector(black_threshold, 50)
        # 存储检测结果
        self.rect_center = None
        self.max_blob = None
        self.target_rect = None
        self.corners = None


    def detect_rect_in_blob(self, img, blob):
        """在色块区域内检测矩形"""
        # 获取色块ROI (x, y, width, height)
        x, y, w, h = blob.rect()
        # 扩展ROI边界，确保矩形完整
        roi = (max(0, x-10), max(0, y-10), min(img.width(), x+w+10), min(img.height(), y+h+10))
        # roi = (max(0, x), max(0, y), min(img.width(), x+w), min(img.height(), y+h))
        # 在ROI内检测矩形（移除max_rects参数）
        rects = img.find_rects(
            threshold=5000,        # 矩形检测阈值
            roi=roi               # 限制检测区域
        )

        if not rects:
            return None

        # 按面积排序，取最大矩形
        largest_rect = max(rects, key=lambda r: r.w() * r.h())
        return largest_rect

    def calculate_center(self, rect):
        """根据矩形角点计算中心点"""
        corners = rect.corners()
        if len(corners) != 4:
            return None, None

        # 计算四个角点的平均坐标
        cx = sum(point[0] for point in corners) // 4
        cy = sum(point[1] for point in corners) // 4
        return corners, (cx, cy)

    def process_frame(self):
        """处理单帧图像，返回矩形四个顶点和中心点"""
        # 读取图像
        img = self.cam.read()
        if img is None:
            return None

        # 步骤1: 寻找最大黑色色块
        _, _, self.max_blob = self.blob_detector.detect_max_blob(img)
        if not self.max_blob:
            return None

        # 步骤2: 在色块内检测矩形
        self.target_rect = self.detect_rect_in_blob(img, self.max_blob)
        if not self.target_rect:
            return None

        # 步骤3: 计算矩形顶点和中心点
        self.corners, self.rect_center = self.calculate_center(self.target_rect)
        if self.rect_center is None:
            return None

        # 检查角度是否接近90度（过滤旋转或不规则矩形）
        angles = calculate_angles(self.corners)  # 需实现角度计算函数
        if all(abs(angle - 90) < 45 for angle in angles):
            return (self.corners, self.rect_center)
        return (None, None)

    import math

def calculate_angles(corners):
    """
    计算矩形四个顶点的夹角
    参数:
        corners: 包含四个顶点坐标的列表，格式为 [(x1,y1), (x2,y2), (x3,y3), (x4,y4)]
    返回:
        angles: 包含四个角度的列表 (单位: 度)
    """
    angles = []
    num_points = len(corners)
    
    for i in range(num_points):
        # 获取当前顶点、前一个顶点和后一个顶点 (通过取模实现循环)
        p_prev = corners[(i - 1) % num_points]
        p_curr = corners[i]
        p_next = corners[(i + 1) % num_points]
        
        # 计算向量 (p_curr -> p_prev) 和 (p_curr -> p_next)
        vec1 = (p_prev[0] - p_curr[0], p_prev[1] - p_curr[1])
        vec2 = (p_next[0] - p_curr[0], p_next[1] - p_curr[1])
        
        # 计算向量点积
        dot_product = vec1[0] * vec2[0] + vec1[1] * vec2[1]
        
        # 计算向量模长
        mag1 = math.sqrt(vec1[0]**2 + vec1[1]**2)
        mag2 = math.sqrt(vec2[0]**2 + vec2[1]**2)
        
        # 避免除零错误
        if mag1 == 0 or mag2 == 0:
            angles.append(0.0)
            continue
        
        # 计算夹角余弦值
        cos_theta = dot_product / (mag1 * mag2)
        
        # 防止数值误差导致的超出范围问题
        cos_theta = max(min(cos_theta, 1.0), -1.0)
        
        # 转换为角度
        theta = math.acos(cos_theta) * (180.0 / math.pi)
        
        # 对于矩形，内角应该接近90度，外角接近270度，但我们只关心内角
        # 取最小角度 (内角和外角之和为360度，取较小的那个)
        theta = min(theta, 360.0 - theta)
        angles.append(theta)
    
    return angles

# 使用示例
if __name__ == "__main__":
    cam = camera.Camera(320, 240, image.Format.FMT_RGB888)
    detector = BlackRectangleDetector(cam, [(0, 0, -128, 0, 0, 0)])
    while not app.need_exit():
        result = detector.process_frame()
        if result:
            corners, center = result
            print(f"顶点: {corners}, 中心点: {center}")
        # time.sleep_ms(10)
    del cam