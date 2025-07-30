from maix import image, display, app, camera
import cv2
import numpy as np

class RectangleDetector:
    def __init__(self, width=320, height=240, canny_threshold1=100, canny_threshold2=200):
        # 初始化摄像头和显示设备
        self.cam = camera.Camera(width, height, image.Format.FMT_BGR888)
        self.disp = display.Display()
        # 边缘检测参数
        self.canny_threshold1 = canny_threshold1
        self.canny_threshold2 = canny_threshold2
        # 存储检测结果
        self.rect_centers = []

    def process_frame(self):
        # 读取摄像头图像
        img = self.cam.read()
        if img is None:
            return None

        # 转换为OpenCV格式
        img_cv = image.image2cv(img, ensure_bgr=False, copy=False)

        # 转为灰度图并进行边缘检测
        gray = cv2.cvtColor(img_cv, cv2.COLOR_BGR2GRAY)
        edges = cv2.Canny(gray, self.canny_threshold1, self.canny_threshold2)

        # 查找轮廓
        contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        self.rect_centers = []

        # 初始化最大矩形变量
        max_area = 0
        max_center = None
        max_approx = None

        # 处理每个轮廓，寻找最大矩形
        for contour in contours:
            # 轮廓近似
            epsilon = 0.02 * cv2.arcLength(contour, True)
            approx = cv2.approxPolyDP(contour, epsilon, True)

            # 判断是否为矩形（4个顶点）且面积大于阈值
            if len(approx) == 4:
                current_area = cv2.contourArea(contour)
                if current_area > 100 and current_area > max_area:
                    # 计算矩形中心点
                    M = cv2.moments(contour)
                    if M["m00"] > 0:
                        cx = int(M["m10"] / M["m00"])
                        cy = int(M["m01"] / M["m00"])
                        # 更新最大矩形信息
                        max_area = current_area
                        max_center = (cx, cy)
                        max_approx = approx

        # 只保留最大矩形的信息
        if max_center is not None:
            self.rect_centers.append(max_center)
            # 绘制最大矩形和中心点
            cv2.drawContours(img_cv, [max_approx], -1, (0, 255, 0), 2)
            cv2.circle(img_cv, max_center, 5, (0, 0, 255), -1)

        # 转换回Maix图像格式并显示
        img_show = image.cv2image(img_cv, bgr=True, copy=False)
        self.disp.show(img_show)

        return self.rect_centers

    def run(self):
        
        # 主循环
        while not app.need_exit():
            self.process_frame()

    def __del__(self):
        # 资源释放
        del self.cam
        del self.disp

# 使用示例
if __name__ == "__main__":
    detector = RectangleDetector()
    detector.run()