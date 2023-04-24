from machine import Pin, PWM, Timer


class MotorController:
    pid_timer = 0

    encoder_pos = 0

    prev_encoder_pos = 0

    current_rpm = 0

    prev_rpm = 0

    target_rpm = 0

    original_rpm = 0

    last_error = 0

    integral_error = 0

    alp = 0.1

    zero_counter = 0

    max_zero_counter = 0

    stuck_rpm = 0

    def __init__(self, dir0, dir1,
                 pwm, enc0, enc1,
                 wheel_diameter, enc_res, gearbox_ratio,
                 pid_interval=20,
                 kp=800, ki=100, kd=200,
                 max_zero_counter=5, stuck_rpm=0.5):

        self.dir0 = Pin(dir0, Pin.OUT)
        self.dir1 = Pin(dir1, Pin.OUT)

        self.pwm = PWM(Pin(pwm, Pin.OUT))

        self.enc0 = Pin(enc0, Pin.IN, Pin.PULL_DOWN)
        self.enc1 = Pin(enc1, Pin.IN, Pin.PULL_DOWN)

        self.kp = kp
        self.ki = ki
        self.kd = kd

        self.pid_interval = pid_interval

        self.wheel_diameter = wheel_diameter
        self.gearbox_ratio = gearbox_ratio
        self.enc_res = enc_res

        self.max_zero_counter = max_zero_counter
        self.stuck_rpm = stuck_rpm

        self.pwm.freq(100)

        self.enc0.irq(trigger=Pin.IRQ_RISING, handler=self.get_position)

    def get_position(self, pin):
        if self.enc1.value():
            self.encoder_pos += 1
        else:
            self.encoder_pos -= 1

    def lp_filter(self, value, prev_value):
        return prev_value * (1 - self.alp) + self.alp * value

    def get_rpm(self):
        # (Encoder Pulses / Encoder Resolution) * (60 / Time of Pulse Measurement) / Gearbox Ratio

        if self.prev_encoder_pos > self.encoder_pos:
            enc_pulses = abs(self.prev_encoder_pos - self.encoder_pos)
        else:
            enc_pulses = abs(self.encoder_pos - self.prev_encoder_pos)

        self.current_rpm = (enc_pulses / self.enc_res) * \
            (60 / (self.pid_interval / 1000)) / self.gearbox_ratio

        self.current_rpm = self.lp_filter(self.current_rpm, self.prev_rpm)

        self.prev_rpm = self.current_rpm

        self.prev_encoder_pos = self.encoder_pos

        return self.current_rpm

    def handle_pid(self, timer):
        current_rpm = self.get_rpm()

        if self.target_rpm > 0 and current_rpm < 0.5:
            self.zero_counter = self.zero_counter + 1

            if self.zero_counter > self.max_zero_counter:
                self.target_rpm = 0
                self.zero_counter = 0
                self.prev_encoder_pos = 0
                self.encoder_pos = 0
                self.current_rpm = 0
                self.integral_error = 0
                self.last_error = 0

                self.pwm.duty_u16(0)

                return

        else:
            self.zero_counter = 0

        error = self.target_rpm - current_rpm
        self.integral_error += error
        derivative = error - self.last_error
        self.last_error = error

        pwm_duty_cycle = int(self.kp * error + self.ki *
                             self.integral_error + self.kd * derivative)

        pwm_duty_cycle = max(min(pwm_duty_cycle, 65535), 0)

        self.pwm.duty_u16(pwm_duty_cycle)

    def set_rpm(self, target_rpm):
        if not self.pid_timer:
            self.pid_timer = Timer(period=self.pid_interval,
                                   mode=Timer.PERIODIC, callback=self.handle_pid)

        if (target_rpm > 0 and self.original_rpm < 0) or (target_rpm < 0 and self.original_rpm > 0):
            return False

        if target_rpm == 0:
            self.dir0.value(0)
            self.dir1.value(0)

        if target_rpm > 0:
            self.dir0.value(0)
            self.dir1.value(1)

        if target_rpm < 0:
            self.dir0.value(1)
            self.dir1.value(0)

        self.original_rpm = target_rpm

        self.target_rpm = abs(target_rpm)

        return True
