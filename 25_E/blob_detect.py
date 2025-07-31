class BlobDetector:
    def __init__(self, threshold, pixels_threshold=1000):
        """
        初始化色块检测器
        :param threshold: 颜色阈值列表，格式为 [(Lmin, Lmax, Amin, Amax, Bmin, Bmax)]
        :param pixels_threshold: 色块最小像素数量阈值
        """
        self.threshold = threshold
        self.pixels_threshold = pixels_threshold
        self.blob_center = (0, 0)  # 色块中心点坐标
        self.image_center = (0, 0)  # 图像中心点坐标
        self.distance = (0, 0)      # (x距离, y距离)
        self.filter_weight = [0.95, 0.05, 0.0, 0.0]  # 权重仅初始化一次
        self.filter_data_x = [0.0] * 4  # 初始化滤波窗口数组
        self.filter_data_y = [0.0] * 4  # 初始化滤波窗口数组
        self.data_pointer_x = 0  # 初始化指针
        self.data_pointer_y = 0  # 初始化指针

    def set_threshold(self, threshold):
        self.threshold = threshold

    def detect_max_blob(self, img, roi=None):
        """
        检测图像中最大的色块并计算距离
        :param img: 输入图像对象
        :return: 成功返回 (中心点坐标, 距离元组)，失败返回 None
        """
        if roi is None:
            roi = (0, 0, img.width(), img.height())  # 默认全图像区域
        # 查找所有符合阈值的色块
        blobs = img.find_blobs(
            self.threshold,
            pixels_threshold=self.pixels_threshold,
            roi=roi
        )

        # 找出最大的色块
        max_blob = None
        max_area = 0
        if blobs:
            for blob in blobs:
                current_area = blob.w() * blob.h()
                if current_area > max_area:
                    max_area = current_area
                    max_blob = blob

        # 计算中心点和距离
        if max_blob:
            # 获取色块中心点
            self.blob_center = (max_blob.cx(), max_blob.cy())
            # 获取图像中心点
            self.image_center = (img.width() // 2, img.height() // 2)
            # 计算距离 (色块中心 - 图像中心)
            self.distance = (
                self.blob_center[0] - self.image_center[0],
                self.blob_center[1] - self.image_center[1]
            )
            return self.blob_center, self.distance, max_blob

        return None, None, None

    def sliding_filter(self, data_x, data_y):
        # 将新数据存入滑动窗口
        self.filter_data_x[self.data_pointer_x] = data_x
        self.filter_data_y[self.data_pointer_y] = data_y
        self.data_pointer_x = (self.data_pointer_x + 1) % 4
        self.data_pointer_y = (self.data_pointer_y + 1) % 4
        
        # 计算加权平均（使用独立变量存储结果）
        filtered_value_x = 0.0
        filtered_value_y = 0.0
        for i in range(4):
            filtered_value_x += self.filter_data_x[i] * self.filter_weight[i]
            filtered_value_y += self.filter_data_y[i] * self.filter_weight[i]
        return int(filtered_value_x), int(filtered_value_y)



""" 帧差法
from maix import image, camera, display, app
import cv2

# 初始化摄像头和显示器
cam = camera.Camera(320, 240)
disp = display.Display()

# 创建形态学操作（膨胀）使用的核
kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))

# 初始化激光点坐标位置变量
point_x = 0
point_y = 0

# 用于存储上一帧灰度图像的变量
last_img_cv_gray = None

# 主循环：直到应用要求退出
while not app.need_exit():
    # 从摄像头读取一帧图像
    img = cam.read()

    # 将maix图像转换为OpenCV格式图像 (BGR格式)
    img_cv = image.image2cv(img, False, False)

    # 将BGR图像转换为灰度图像
    img_cv_gray = cv2.cvtColor(img_cv, cv2.COLOR_BGR2GRAY)

    # 如果是第一帧，则保存为“上一帧灰度图像”
    if last_img_cv_gray is None:
        last_img_cv_gray = img_cv_gray.copy()

    # 计算当前帧灰度图与上一帧灰度图的绝对值差（帧间差分）
    img_diff = cv2.absdiff(img_cv_gray, last_img_cv_gray)

    # 对差分图像进行二值化处理（阈值设为25）
    _, img_binary = cv2.threshold(img_diff, 25, 255, cv2.THRESH_BINARY)

    # 使用膨胀处理填充激光点内部的空洞（进行2次迭代）
    img_binary = cv2.dilate(img_binary, kernel, iterations=2)

    # check point 放开注释,应能黑色背景下激光点二值化后的白点
    # img_show = image.cv2image(img_binary, False, False)
    # disp.show(img_show)

    # 查找轮廓
    contours, _ = cv2.findContours(img_binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    point_x, point_y = 0, 0  # 初始化坐标

    for contour in contours:
        # 计算轮廓面积
        contour_area = cv2.contourArea(contour)
        
        # 激光点不大，过滤掉大范围的轮廓
        if contour_area < 500:
            # 计算轮廓质心作为激光点位置坐标
            M = cv2.moments(contour)
            if M["m00"] != 0:  # 防止除零错误
                point_x = int(M["m10"] / M["m00"])
                point_y = int(M["m01"] / M["m00"])
            
            # 获取激光点轮廓的外接矩形
            x, y, w, h = cv2.boundingRect(contour)
            # cv2.rectangle(img_cv, (x, y), (x + w, y + h), (0, 255, 0), 2)
            
            # 获取该区域的直方图统计数据，以区分红色激光点或者绿色激光点
            hist = img.get_histogram(thresholds=[[0, 100, -128, 127, -128, 127]], 
                                    roi=[x, y, w, h])
            value = hist.get_statistics().a_median()  # 获取颜色中值
            print('staticstic value = {} and contour area = {}'.format(value, contour_area))

    last_img_cv_gray = img_cv_gray.copy()

    img.draw_cross(point_x, point_y, image.COLOR_BLUE, 5, 2)
    disp.show(img)
"""
