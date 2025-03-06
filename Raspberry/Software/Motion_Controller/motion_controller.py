import os
import RPi.GPIO as GPIO
import pygame
import numpy as np
from pygame.math import Vector2

def map_range(value, in_min, in_max, out_min, out_max):
    return out_min + (float(value - in_min) / (in_max - in_min)) * (out_max - out_min)

def main():
    os.environ["SDL_VIDEODRIVER"] = "dummy"

    pygame.init()
    pygame.joystick.init()

    joystick = pygame.joystick.Joystick(0)
    joystick.init()
    JOYSTICK_THRESHOLD = 0.99
    adjusted_angle = None
    speed_const = 

    GPIO.setmode(GPIO.BCM)
    
    CHNAGE_DIR_PIN1 = 5
    CHNAGE_DIR_PIN2 = 6
    CHNAGE_DIR_PIN3 = 26
    PWM_PIN1 = 12
    PWM_PIN2 = 13
    PWM_PIN3 = 19

    SERVO_PIN1 = 17
    SERVO_PIN2 = 22
    SERVO_PIN3 = 24
    SERVO_PIN4 = 25

    BUTTON_PIN = 5
    DUTY_CYCLE = 0  

    GPIO.setup([PWM_PIN1, PWM_PIN2, PWM_PIN3], GPIO.OUT)
    GPIO.setup(BUTTON_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)

    pwm1 = GPIO.PWM(PWM_PIN1, 1000)  
    pwm1.start(DUTY_CYCLE)

    pwm2 = GPIO.PWM(PWM_PIN2, 1000)  
    pwm2.start(DUTY_CYCLE)

    pwm3 = GPIO.PWM(PWM_PIN3, 1000)  
    pwm3.start(DUTY_CYCLE)

    GPIO.setup([CHNAGE_DIR_PIN1, CHNAGE_DIR_PIN2, CHNAGE_DIR_PIN3], GPIO.OUT)

    freq = 50
    
    GPIO.setup([SERVO_PIN1, SERVO_PIN2, SERVO_PIN3, SERVO_PIN4], GPIO.OUT)
    servo1 = GPIO.PWM(SERVO_PIN1, freq) 
    servo2 = GPIO.PWM(SERVO_PIN2, freq)
    servo3 = GPIO.PWM(SERVO_PIN3, freq)
    servo4 = GPIO.PWM(SERVO_PIN4, freq)

    servo1.start(7.5)  # Neutral position ???
    servo2.start(7.5)
    servo3.start(7.5)
    servo4.start(7.5)

    V_left = 0
    V_right = 0
    V_back = 0
    rt_value = -1
    lt_value = -1

    matrix = np.array([[-0.33, 0.58, 0.33],
                        [-0.33, -0.58, 0.33],
                        [0.67, 0, 0.33]])

    try:
        while True:
            while GPIO.input(BUTTON_PIN) == GPIO.LOW:
                lb_value = joystick.get_button(6)
                rb_value = joystick.get_button(7)  # RB button ???

                for event in pygame.event.get():
                    if event.type == pygame.QUIT:
                        return

                if rb_value:  
                    left_x = joystick.get_axis(0)
                    left_y = joystick.get_axis(1)
                    right_x = joystick.get_axis(2)
                    right_y = joystick.get_axis(3)

                    servo1_angle = map_range(left_x, -1, 1, 2.5, 12.5)
                    servo2_angle = map_range(left_y, -1, 1, 2.5, 12.5)
                    servo3_angle = map_range(right_x, -1, 1, 2.5, 12.5)
                    servo4_angle = map_range(right_y, -1, 1, 2.5, 12.5)

                    servo1.ChangeDutyCycle(servo1_angle)
                    servo2.ChangeDutyCycle(servo2_angle)
                    servo3.ChangeDutyCycle(servo3_angle)
                    servo4.ChangeDutyCycle(servo4_angle)

                elif lb_value:  # Motor control mode
                    lt_value = joystick.get_axis(5)
                    rt_value = joystick.get_axis(4)
                    right_joystick_y = -(joystick.get_axis(2))
                    right_joystick_x = -(joystick.get_axis(3))

                    OldRange = (1 - 0.15)
                    NewRange = (100 - 1)
                    speed_const = (((max(abs(right_joystick_y), abs(right_joystick_x)) - 0.15) * NewRange) / OldRange) + 1

                    vec = Vector2(right_joystick_x, right_joystick_y)
                    magnitude = vec.length()
                    adjusted_angle = None if magnitude < 0.15 else int((vec.as_polar()[1] + 90) % 360)

                    if lt_value > -0.98:
                        direction = np.array([0, 0, 1] if adjusted_angle is None else [np.cos(np.radians(adjusted_angle)), np.sin(np.radians(adjusted_angle)), 1])
                        speed_const = (((lt_value - -1) * NewRange) / OldRange) + 1
                    elif rt_value > -0.98:
                        direction = np.array([0, 0, -1] if adjusted_angle is None else [np.cos(np.radians(adjusted_angle)), np.sin(np.radians(adjusted_angle)), -1])
                        speed_const = (((rt_value - -1) * NewRange) / OldRange) + 1
                    elif adjusted_angle is not None:
                        direction = np.array([np.cos(np.radians(adjusted_angle)), np.sin(np.radians(adjusted_angle)), 0])

                    result = np.dot(matrix, direction)
                    V_left, V_right, V_back = -speed_const * result[1], -speed_const * result[0], -speed_const * result[2]

                    for pin, pwm, value in zip([CHNAGE_DIR_PIN3, CHNAGE_DIR_PIN2, CHNAGE_DIR_PIN1], [pwm3, pwm2, pwm1], [V_left, V_right, V_back]):
                        GPIO.setup(pin, GPIO.OUT if value < 0 else GPIO.IN)
                        pwm.ChangeDutyCycle(np.abs(round(value, 2)))

                else:
                    pwm1.ChangeDutyCycle(0)
                    pwm2.ChangeDutyCycle(0)
                    pwm3.ChangeDutyCycle(0)
                    servo1.ChangeDutyCycle(7.5)
                    servo2.ChangeDutyCycle(7.5)
                    servo3.ChangeDutyCycle(7.5)
                    servo4.ChangeDutyCycle(7.5)

            print("Stop button is pressed...")

    except KeyboardInterrupt:
        print("Exiting...")
    finally:
        pygame.quit()
        pwm1.stop()
        pwm2.stop()
        pwm3.stop()
        servo1.stop()
        servo2.stop()
        servo3.stop()
        servo4.stop()
        GPIO.cleanup()


if __name__ == "__main__":
    main()
