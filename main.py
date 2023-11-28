#!/usr/bin/env python3
from ev3dev2.motor import SpeedPercent, MoveTank, MediumMotor
from ev3dev2.sensor.lego import ColorSensor, TouchSensor
from ev3dev2.motor import OUTPUT_A, OUTPUT_B, OUTPUT_C
from ev3dev2.sensor import INPUT_2, INPUT_3, INPUT_4


class LineFollower:
    def __init__(
        self,
        motor_out1,
        motor_out2,
        grabber_out,
        color_sensor_left_in,
        color_sensor_right_in,
        touch_sensor_in,
        follower_speed_percent,
        box_mode_speed_percent,
        grabber_speed,
        color_threshold_white,
        P,
        I,
        D,
        box_field_color,
    ):
        self._move_tank = MoveTank(motor_out1, motor_out2)
        self._grabber = MediumMotor(grabber_out)
        self._color_sensor_left = ColorSensor(color_sensor_left_in)
        self._color_sensor_right = ColorSensor(color_sensor_right_in)
        self._touch_sensor = TouchSensor(touch_sensor_in)
        self._follower_speed_percent = follower_speed_percent
        self._box_mode_speed_percent = box_mode_speed_percent
        self._grabber_speed = grabber_speed
        self._color_threshold_white = color_threshold_white
        self._holding_box = False
        self._task_done = False
        self._K_p = P
        self._K_i = I
        self._K_d = D
        self._last_error = 0
        self._integral = 0
        self._derivative = 0
        self._left_gray_scale = 0
        self._right_gray_scale = 0
        self._box_field_color = box_field_color

    def follow_line(self):
        error = self.calculate_error()
        command = self.compute_command(error)
        speed1 = self._follower_speed_percent - command
        speed2 = self._follower_speed_percent + command
        self._move_tank.on(
            SpeedPercent(self.enforce_boundaries(speed1)),
            SpeedPercent(self.enforce_boundaries(speed2)),
        )

    def handle_the_box(self, color):
        if self.check_color(self._color_sensor_left.rgb, color):
            self.move_forward(1.2)
            self.turn_degrees(90, left=True)
            turned_left = True
        elif self.check_color(self._color_sensor_right.rgb, color):
            self.move_forward(1.2)
            self.turn_degrees(90, left=False)
            turned_left = False
        else:
            return

        self.line_follow_until_color([color])
        # Turn around and place touch sensor at the beginning of the field
        self.move_forward(-4)
        self.turn_degrees(180)

        if self._holding_box:
            self.put_box_down()
        else:
            self.find_and_grab_box()

        self.line_follow_until_color(["black"])
        self.move_forward(1.2)
        self.turn_degrees(90, turned_left)

    def find_and_grab_box(self):
        FIELD_WIDTH = 4
        FIELD_LENGTH = 4
        STEPS_HORIZONTAL = 3
        STEPS_VERTICAL = 10

        # Search from right side of the field to the left side
        self.move_left_right(FIELD_WIDTH / 2, left_direction=True)
        for i in range(STEPS_HORIZONTAL):
            for j in range(STEPS_VERTICAL):
                self.move_forward(-FIELD_LENGTH / STEPS_VERTICAL)
                if self._touch_sensor.is_pressed:
                    # Grab box
                    self.move_forward(2)
                    self.grabber_on(up=False)
                    self.move_forward(-1)
                    self.grabber_on(up=True)
                    self._holding_box = True
                    # Move back vertically
                    self.move_forward((j + 1) * FIELD_LENGTH / STEPS_VERTICAL)
                    break
            self.move_forward(FIELD_LENGTH)
            if self._holding_box:
                # Move back horizontally
                self.move_left_right(i - FIELD_WIDTH / 2, left_direction=True)
                break
            else:
                self.move_left_right(
                    FIELD_WIDTH / STEPS_HORIZONTAL, left_direction=False
                )
                continue

        if not self._holding_box:
            self.move_left_right(FIELD_WIDTH / 2, left_direction=False)

    def put_box_down(self):
        self.move_forward(-1)
        self.grabber_on(up=False)
        self.move_forward(2.5)
        self.grabber_on(up=True)
        self._holding_box = False
        self._task_done = True

    def check_color(self, rgb, color):
        if color == "red":
            if (rgb[0] > 160) and (rgb[1] < 60) and (rgb[2] < 30):
                return True
        elif color == "green":
            if (rgb[0] < 40) and (rgb[1] > 130) and (rgb[2] < 60):
                return True
        elif color == "blue":
            if (rgb[0] < 40) and (rgb[1] < 30) and (rgb[2] > 150):
                return True
        elif color == "black":
            if (rgb[0] < 50) and (rgb[1] < 50) and (rgb[2] < 50):
                return True
        elif color == "white":
            if (rgb[0] > 150) and (rgb[1] > 150) and (rgb[2] > 150):
                return True
        return False

    def turn_degrees(self, angle, left=False):
        self.stop()
        MULT = 22 / 9
        if left is True:
            # rotate left
            self._move_tank.on_for_degrees(
                SpeedPercent(-self._box_mode_speed_percent),
                SpeedPercent(self._box_mode_speed_percent),
                MULT * angle,
            )
        else:
            # rotate right
            self._move_tank.on_for_degrees(
                SpeedPercent(self._box_mode_speed_percent),
                SpeedPercent(-self._box_mode_speed_percent),
                MULT * angle,
            )

    def move_forward(self, units):
        self.stop()
        self._move_tank.on_for_degrees(
            SpeedPercent(self._box_mode_speed_percent),
            SpeedPercent(self._box_mode_speed_percent),
            100 * units,
        )

    def move_left_right(self, units, left_direction):
        self.stop()
        self.turn_degrees(90, left_direction)
        self.move_forward(units)
        self.turn_degrees(90, not left_direction)

    def grabber_on(self, up=False):
        if up:
            self._grabber.on_for_degrees(SpeedPercent(self._grabber_speed), 70)
        else:
            self._grabber.on_for_degrees(SpeedPercent(-self._grabber_speed), 70)

    def rgb_to_gray(self, rgb):
        r, g, b = rgb
        gray_value = 0.299 * r + 0.587 * g + 0.114 * b
        return int(gray_value)

    def compute_command(self, error):
        self._integral += error
        self._derivative = error - self._last_error
        self._last_error = error
        return (
            self._K_p * error
            + self._K_i * self._integral
            + self._K_d * self._derivative
        )

    def calculate_error(self):
        error = 0
        self._left_gray_scale = self.rgb_to_gray(self._color_sensor_left.rgb)
        self._right_gray_scale = self.rgb_to_gray(self._color_sensor_right.rgb)

        if self._left_gray_scale < self._color_threshold_white:
            error += self._color_threshold_white - self._left_gray_scale

        if self._right_gray_scale < self._color_threshold_white:
            error -= self._color_threshold_white - self._right_gray_scale

        return error

    def line_follow_until_color(self, color_stop_list):
        follow_line = True
        try:
            while follow_line:
                self.follow_line()
                for color in color_stop_list:
                    if self.check_color(
                        self._color_sensor_left.rgb, color
                    ) and self.check_color(self._color_sensor_right.rgb, color):
                        follow_line = False
        except KeyboardInterrupt:
            self.stop()

        self.stop()
        return

    def enforce_boundaries(self, value):
        if value > 100:
            return 100
        elif value < -100:
            return -100
        else:
            return value

    def stop(self):
        self._move_tank.stop()

    def run(self):
        try:
            if not self._task_done:
                while True:
                    self.follow_line()
                    self.handle_the_box(self._box_field_color)
            else:
                self.stop()
        except KeyboardInterrupt:
            self.stop()


if __name__ == "__main__":
    line_follower = LineFollower(
        motor_out1=OUTPUT_A,
        motor_out2=OUTPUT_B,
        grabber_out=OUTPUT_C,
        color_sensor_left_in=INPUT_3,
        color_sensor_right_in=INPUT_2,
        touch_sensor_in=INPUT_4,
        follower_speed_percent=12,
        box_mode_speed_percent=15,
        grabber_speed=10,
        color_threshold_white=220,
        P=0.1,
        I=0,
        D=0.02,
        box_field_color="green",
    )
    line_follower.run()
