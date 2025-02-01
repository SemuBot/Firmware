import os
import RPi.GPIO as GPIO
import pygame
import numpy as np
from pygame.math import Vector2
def main():
    os.environ["SDL_VIDEODRIVER"] = "dummy"
    
    pygame.init()
    pygame.joystick.init()

    joystick = pygame.joystick.Joystick(0)
    joystick.init()

    JOYSTICK_THRESHOLD = 0.99
    adjusted_angle = None
    speed_const = 100

    GPIO.setmode(GPIO.BCM)
    CHNAGE_DIR_PIN1 = 23
    CHNAGE_DIR_PIN2 = 6
    CHNAGE_DIR_PIN3 = 4
    PWM_PIN1 = 18
    PWM_PIN2 = 13
    PWM_PIN3 = 27

    DUTY_CYCLE = 0  

    GPIO.setup(PWM_PIN1, GPIO.OUT)
    GPIO.setup(PWM_PIN2, GPIO.OUT)
    GPIO.setup(PWM_PIN3, GPIO.OUT)
    pwm1 = GPIO.PWM(PWM_PIN1, 1000)  
    pwm1.start(DUTY_CYCLE)

    pwm2 = GPIO.PWM(PWM_PIN2, 1000)  
    pwm2.start(DUTY_CYCLE)

    pwm3 = GPIO.PWM(PWM_PIN3, 1000)  
    pwm3.start(DUTY_CYCLE)


    GPIO.setup(CHNAGE_DIR_PIN1, GPIO.OUT)
    GPIO.setup(CHNAGE_DIR_PIN2, GPIO.OUT)
    GPIO.setup(CHNAGE_DIR_PIN3, GPIO.OUT)
    V_left = 0
    V_right = 0
    V_back = 0
    rt_value = -1
    lt_value = -1
    
    try:

        
        while True:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    return

            lt_value = joystick.get_axis(5)
            rt_value = joystick.get_axis(4)

            right_joystick_y = -(joystick.get_axis(3)) 
            right_joystick_x = -(joystick.get_axis(2))  

            OldRange = (1 - 0.15)  
            NewRange = (100 - 1)  
            speed_const = (((max(abs(right_joystick_y), abs(right_joystick_x)) - 0.15) * NewRange) / OldRange) + 1
            #print(right_joystick_x)
            vec = Vector2(right_joystick_x, right_joystick_y)
            magnitude = vec.length()
            if abs(right_joystick_y) < 0.15 and abs(right_joystick_x) < 0.15:
                adjusted_angle = None
            else:
                radius, angle = vec.as_polar()
                adjusted_angle = int((angle-90) % 360)
            #print(adjusted_angle, rt_value, lt_value)

            if lt_value > -0.98:

                if adjusted_angle != None:
                    speed_const = 50
                    OldRange = (1 - -1)  
                    NewRange = (2 - 0)  
                    rot_multiplier = (((lt_value - -1) * NewRange) / OldRange) + 0

                    V_left = speed_const * np.cos(np.radians(30) - np.radians(adjusted_angle)) + -(rot_multiplier * 25)
                    V_right = speed_const * np.cos(np.radians(150) - np.radians(adjusted_angle)) + -(rot_multiplier * 25)
                    V_back = speed_const * np.cos(np.radians(270) - np.radians(adjusted_angle)) + -(rot_multiplier * 25)

                    
                else:

                    speed_const = 100

                    OldRange = (1 - -1)  
                    NewRange = (1 - 0)  
                    rot_multiplier = (((lt_value - -1) * NewRange) / OldRange) + 0
                    #print(rot_multiplier)

                    V_left = -(speed_const * rot_multiplier)
                    V_right = -(speed_const * rot_multiplier)
                    V_back = -(speed_const * rot_multiplier)

            elif rt_value > -0.98:

                if adjusted_angle != None:

                    speed_const = 50
                    OldRange = (1 - -1)  
                    NewRange = (2 - 0)  
                    rot_multiplier = (((rt_value - -1) * NewRange) / OldRange) + 0

                    V_left = speed_const * np.cos(np.radians(30) - np.radians(adjusted_angle)) + (rot_multiplier * 25)
                    V_right = speed_const * np.cos(np.radians(150) - np.radians(adjusted_angle)) + (rot_multiplier * 25)
                    V_back = speed_const * np.cos(np.radians(270) - np.radians(adjusted_angle)) + (rot_multiplier * 25)
                else:
                    speed_const = 100

                    OldRange = (1 - -1)  
                    NewRange = (1 - 0)  
                    rot_multiplier = (((rt_value - -1) * NewRange) / OldRange) + 0
                    #print(rot_multiplier)

                    V_left = (speed_const * rot_multiplier)
                    V_right = (speed_const * rot_multiplier)
                    V_back = (speed_const * rot_multiplier)
            
            elif rt_value == -1 and lt_value == -1 and adjusted_angle != None:
                
                speed_const = 100

                V_left = (speed_const * np.cos(np.radians(30) - np.radians(adjusted_angle))) 
                V_right = (speed_const * np.cos(np.radians(150) - np.radians(adjusted_angle)))
                V_back = (speed_const * np.cos(np.radians(270) - np.radians(adjusted_angle)))


            if rt_value != -1 or lt_value != -1 or adjusted_angle != None:

                if V_left < 0:
                    GPIO.setup(CHNAGE_DIR_PIN3, GPIO.OUT)
                    GPIO.output(CHNAGE_DIR_PIN3, GPIO.LOW)
                    pwm3.ChangeDutyCycle(np.abs(round(V_left, 2)))
                else:
                    GPIO.setup(CHNAGE_DIR_PIN3, GPIO.IN)
                    pwm3.ChangeDutyCycle(np.abs(round(V_left, 2)))

                if V_right < 0:
                    GPIO.setup(CHNAGE_DIR_PIN2, GPIO.OUT)
                    GPIO.output(CHNAGE_DIR_PIN2, GPIO.LOW)
                    pwm2.ChangeDutyCycle(np.abs(round(V_right, 2)))
                else:
                    GPIO.setup(CHNAGE_DIR_PIN2, GPIO.IN)
                    pwm2.ChangeDutyCycle(np.abs(round(V_right, 2)))

                if V_back < 0:
                    GPIO.setup(CHNAGE_DIR_PIN1, GPIO.OUT)
                    GPIO.output(CHNAGE_DIR_PIN1, GPIO.LOW)
                    pwm1.ChangeDutyCycle(np.abs(round(V_back, 2)))
                else:
                    GPIO.setup(CHNAGE_DIR_PIN1, GPIO.IN)
                    pwm1.ChangeDutyCycle(np.abs(round(V_back, 2)))
            else:
                V_left = 0
                V_right = 0
                V_back = 0
                pwm1.ChangeDutyCycle(0)
                pwm2.ChangeDutyCycle(0)
                pwm3.ChangeDutyCycle(0)

            print(V_left, V_right, V_back)

    except KeyboardInterrupt:
        print("Exiting...")
    finally:
        pygame.quit()
        pwm1.stop()
        pwm2.stop()
        pwm3.stop()
        GPIO.cleanup()


if __name__ == "__main__":
    main()
