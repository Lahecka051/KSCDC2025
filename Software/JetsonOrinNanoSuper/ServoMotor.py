from machine import PWM

class Servo:

    def __init__(self, pin, freq = 50, min_us=500, max_us=2500, angle=180):
        self.min_us = min_us
        self.max_us = max_us
        self.freq = freq
        self.angle = angle
        self.pwm = PWM(pin, freq=freq, duty=0)

    def servo_angle(self, degrees):
        degrees = degrees % 360
        total_range = self.max_us - self.min_us
        us = self.min_us + total_range * degrees // self.angle
        ns = us * 1000
        self.pwm.duty_ns(ns)
