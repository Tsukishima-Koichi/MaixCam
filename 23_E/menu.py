from maix import touchscreen, display, image, camera

class MenuInterface:
    def __init__(self, disp, ts, cam):
        # 初始化硬件设备
        self.ts = ts
        self.disp = disp
        
        # 状态标志
        self.red_flag = False
        self.blue_flag = False
        self.start_flag = False
        
        # UI元素属性
        self.cam = cam
        self.img = image.Image(320, 240)  # 使用空白缓冲区初始化
        self.img.draw_rect(0, 0, self.img.width(), self.img.height(), image.COLOR_WHITE)
        self.red_btn_pos = None
        self.blue_btn_pos = None
        self.start_btn_pos = None
        self.red_btn_disp_pos = None
        self.blue_btn_disp_pos = None
        self.start_btn_disp_pos = None
        
        # 初始化界面
        self._init_ui()

    def _init_ui(self):
        # 创建图像缓冲区

        self.img.draw_rect(0, 0, self.img.width(), self.img.height(), image.COLOR_WHITE)

        start_label = "S"
        start_size = image.string_size(start_label)
        self.start_btn_pos = [0, 0, 8*2 + start_size.width(), 12*2 + start_size.height()]
        self.img.draw_string(8, 12, start_label, image.COLOR_WHITE)
        self.img.draw_rect(*self.start_btn_pos, image.COLOR_WHITE, 2)
        
        # 绘制红色按钮
        red_label = "R"
        red_size = image.string_size(red_label)
        self.red_btn_pos = [0, 12*2 + start_size.height(), 8*2 + red_size.width(), 12*2 + red_size.height()]
        self.img.draw_string(8, 12*3 + start_size.height(), red_label, image.COLOR_WHITE)
        self.img.draw_rect(*self.red_btn_pos, image.COLOR_WHITE, 2)
        
        # 绘制蓝色按钮
        blue_label = "B"
        blue_size = image.string_size(blue_label)
        self.blue_btn_pos = [0, 12*4 + red_size.height() + start_size.height(), 8*2 + blue_size.width(), 12*2 + blue_size.height()]
        self.img.draw_string(8, 12*5 + red_size.height() + start_size.height(), blue_label, image.COLOR_WHITE)
        self.img.draw_rect(*self.blue_btn_pos, image.COLOR_WHITE, 2)
        
        # 计算屏幕坐标映射
        self._update_button_positions()

    def _update_button_positions(self):
        # 更新按钮在显示屏上的实际坐标
        self.start_btn_disp_pos = image.resize_map_pos(
            self.img.width(), self.img.height(),
            self.disp.width(), self.disp.height(),
            image.Fit.FIT_CONTAIN, *self.start_btn_pos
        )
        self.red_btn_disp_pos = image.resize_map_pos(
            self.img.width(), self.img.height(),
            self.disp.width(), self.disp.height(),
            image.Fit.FIT_CONTAIN, *self.red_btn_pos
        )
        self.blue_btn_disp_pos = image.resize_map_pos(
            self.img.width(), self.img.height(),
            self.disp.width(), self.disp.height(),
            image.Fit.FIT_CONTAIN, *self.blue_btn_pos
        )

    def is_in_button(self, x, y, btn_pos):
        # 检查触摸点是否在按钮区域内
        return (x > btn_pos[0] and x < btn_pos[0] + btn_pos[2] and
                y > btn_pos[1] and y < btn_pos[1] + btn_pos[3])

    def update(self):
        # 处理触摸输入并更新状态
        x, y, pressed = self.ts.read()
        if pressed:
            self.red_flag = self.is_in_button(x, y, self.red_btn_disp_pos)
            self.blue_flag = self.is_in_button(x, y, self.blue_btn_disp_pos)
            self.start_flag = self.is_in_button(x, y, self.start_btn_disp_pos)
        return self.red_flag, self.blue_flag, self.start_flag

    def render(self, background_img=None):
        
        if background_img:
            background_img.draw_rect(0, 0, background_img.width(), background_img.height(), image.COLOR_WHITE)

            start_label = "S"
            start_size = image.string_size(start_label)
            background_img.draw_string(8, 12, start_label, image.COLOR_WHITE)
            background_img.draw_rect(*self.start_btn_pos, image.COLOR_WHITE, 2)

            # 绘制蓝色按钮
            
            blue_label = "B"
            blue_size = image.string_size(blue_label)
            background_img.draw_string(8, 12*3 + start_size.height(), blue_label, image.COLOR_WHITE)
            background_img.draw_rect(*self.blue_btn_pos, image.COLOR_WHITE, 2)

            # 绘制红色按钮
            red_label = "R"
            red_size = image.string_size(red_label)
            background_img.draw_string(8, 12*5 + blue_size.height() + start_size.height(), red_label, image.COLOR_WHITE)
            background_img.draw_rect(*self.red_btn_pos, image.COLOR_WHITE, 2)
            
            
        else:
            self.disp.show(self.img, fit=image.Fit.FIT_CONTAIN)

    def get_flags(self):
        # 获取当前按钮状态
        return self.red_flag, self.blue_flag, self.start_flag






# 使用示例
if __name__ == "__main__":
    disp = display.Display()
    ts = touchscreen.TouchScreen()

    menu = MenuInterface(disp, ts)
    while True:
        menu.update()
        menu.render()
        red, blue = menu.get_flags()
        # 在这里添加状态处理逻辑
