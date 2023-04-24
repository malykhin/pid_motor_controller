from machine import Pin
import time
import select
import sys
import json

from motor_controller import MotorController

stby = Pin(22, Pin.OUT)

# PWM 28
# DIR0 26
# DIR1 27
# ENC0 5
# ENC1 6

# PWM 19
# DIR0 20
# DIR1 21
# ENC0 7
# ENC1 8

# PWM 14
# DIR0 13
# DIR1 15
# ENC0 11
# ENC1 12


# PWM 18
# DIR0 17
# DIR1 16
# ENC0 9
# ENC1 10

enc_res = 7
gearbox_ratio = 86
wheel_diameter = 0.0047

is_line_available = False
line = ""

if __name__ == "__main__":
    stby.value(1)

    motor0 = MotorController(dir0=26, dir1=27, pwm=28,
                             enc0=5, enc1=6, wheel_diameter=wheel_diameter,
                             enc_res=enc_res, gearbox_ratio=gearbox_ratio,
                             )

    motor1 = MotorController(dir0=20, dir1=21, pwm=19,
                             enc0=7, enc1=8, wheel_diameter=wheel_diameter,
                             enc_res=enc_res, gearbox_ratio=gearbox_ratio,
                             )

    motor2 = MotorController(dir0=14, dir1=15, pwm=13,
                             enc0=11, enc1=12, wheel_diameter=wheel_diameter,
                             enc_res=enc_res, gearbox_ratio=gearbox_ratio,
                             )

    motor3 = MotorController(dir0=16, dir1=17, pwm=18,
                             enc0=9, enc1=10, wheel_diameter=wheel_diameter,
                             enc_res=enc_res, gearbox_ratio=gearbox_ratio,
                             )

    while True:
        if is_line_available:
            try:
                speed0, speed1, speed2, speed3 = json.loads(line)['speeds']

                res0 = motor0.set_rpm(speed0)
                res1 = motor1.set_rpm(speed1)
                res2 = motor2.set_rpm(speed2)
                res3 = motor3.set_rpm(speed3)

                answer = {"results": [res0, res1, res2, res3]}

                print(json.dumps(answer))

            except:
                motor0.set_rpm(0)
                motor1.set_rpm(0)
                motor2.set_rpm(0)
                motor3.set_rpm(0)

                answer = {"results": [False, False, False, False]}

                print(json.dumps(answer))

            finally:
                is_line_available = False
                line = ""

        while sys.stdin in select.select([sys.stdin], [], [], 0)[0]:
            ch = sys.stdin.read(1)
            if ch == '\n':
                is_line_available = True
            else:
                line = line + ch
