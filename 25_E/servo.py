from maix import pwm, time, pinmap, app

class ServoController:
    # 舵机PWM参数（类级常量）
    SERVO_PERIOD = 50      # PWM周期（Hz）
    SERVO_MIN_DUTY = 2.5   # 最小占空比（2.5% -> 0.5ms）
    SERVO_MAX_DUTY = 12.5  # 最大占空比（12.5% -> 2.5ms）
    ANGLE_180_RANGE = 180      # 默认角度范围（0-180度）
    ANGLE_270_RANGE = 270      # 默认角度范围（0-180度）
    SERVO_180_LIMIT = (10.0, 180.0)
    SERVO_270_LIMIT = (60.0, 210.0)

    def __init__(self, max_angle):
        """
        初始化舵机控制器
        :param pwm_id: PWM通道ID
        :param pin_name: 舵机连接引脚
        :param max_angle: 最大可控角度
        """
        if max_angle == 180:
            self.pwm_id = 7
            self.pin_name = "A19"
            self.current_angle = 110
        elif max_angle == 270:
            self.pwm_id = 6
            self.pin_name = "A18"
            self.current_angle = max_angle / 2
        
        self.min_angle = 0
        self.max_angle = max_angle
        
        
        # 配置引脚功能
        pinmap.set_pin_function(self.pin_name, f"PWM{self.pwm_id}")

        # 初始化PWM输出
        self.pwm = pwm.PWM(
            self.pwm_id,
            freq=self.SERVO_PERIOD,
            duty=self._angle_to_duty(self.current_angle),
            enable=True
        )

    def _angle_to_duty(self, angle):
        """将角度转换为PWM占空比"""
        # 角度范围映射到0-100%（百分比）
        percent = (angle - self.min_angle) / (self.max_angle - self.min_angle) * 100
        # 百分比转换为占空比
        return (self.SERVO_MAX_DUTY - self.SERVO_MIN_DUTY) * percent / 100.0 + self.SERVO_MIN_DUTY

    def set_angle(self, angle):
        """
        设置舵机角度
        :param angle: 目标角度（度）
        """
        # 角度限幅
        if self.max_angle == 270 :
            target_angle = max(self.SERVO_270_LIMIT[0], min(angle, self.SERVO_270_LIMIT[1])) 
        else:
            target_angle = max(self.SERVO_180_LIMIT[0], min(angle, self.SERVO_180_LIMIT[1]))

        # 设置最终角度
        self.pwm.duty(self._angle_to_duty(target_angle))
        self.current_angle = target_angle

    def stop(self):
        """停止舵机PWM输出"""
        self.pwm.enable(False)

# 使用示例
if __name__ == "__main__":
    # 创建舵机控制器实例（默认0-180度）
    servo = ServoController(270)

    # while app.need_exit():
    #     # 直接设置角度（立即转动）
    #     servo.set_angle(45)  # 转到90度

    servo.set_angle(135)  # 2秒内转到180度

    # 清理资源
    # servo.stop()




# // ... existing code ...
#     def __init__(self, max_angle):
#         // ... existing code ...
#         self.current_angle = 0
#         # 平滑转动状态变量
#         self._is_smoothing = False
#         self._start_angle = 0
#         self._target_angle = 0
#         self._start_time = 0
#         self._smooth_duration = 0
#         // ... existing code ...

#     def set_angle(self, angle, smooth_time=0):
#         """
#         设置舵机角度
#         :param angle: 目标角度（度）
#         :param smooth_time: 平滑转动时间（毫秒），0为立即转动
#         """
#         target_angle = max(self.min_angle, min(angle, self.max_angle))

#         if smooth_time <= 0:
#             # 立即转动
#             self.pwm.duty(self._angle_to_duty(target_angle))
#             self.current_angle = target_angle
#             self._is_smoothing = False
#         else:
#             # 初始化平滑转动参数
#             self._is_smoothing = True
#             self._start_angle = self.current_angle
#             self._target_angle = target_angle
#             self._start_time = time.ticks_ms()
#             self._smooth_duration = smooth_time

#     def update_smooth_movement(self):
#         """更新平滑转动状态（需在主循环中定期调用）"""
#         if not self._is_smoothing:
#             return False

#         current_time = time.ticks_ms()
#         elapsed = time.ticks_diff(current_time, self._start_time)

#         if elapsed >= self._smooth_duration:
#             # 平滑转动结束
#             self.pwm.duty(self._angle_to_duty(self._target_angle))
#             self.current_angle = self._target_angle
#             self._is_smoothing = False
#             return False
#         else:
#             # 计算当前角度（线性插值）
#             progress = elapsed / self._smooth_duration
#             current_angle = self._start_angle + (self._target_angle - self._start_angle) * progress
#             self.pwm.duty(self._angle_to_duty(current_angle))
#             self.current_angle = current_angle
#             return True

#     def stop(self):
#         // ... existing code ...

# # 初始化舵机
#  servo = Servo(max_angle=180)
# # 设置目标角度（1秒内平滑转动）
#  servo.set_angle(90, smooth_time=1000)

# # 主循环
# while True:
#     # 定期更新舵机状态
#     servo.update_smooth_movement()
#     # 其他任务...