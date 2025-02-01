import RPi.GPIO as GPIO
import time
import os
import subprocess

screen_session = "controller"
controller_code = "/home/semubot/Software/Motion_Controller/motion_controller_w_matrix.py"

GPIO.setmode(GPIO.BCM)
BUTTON_PIN = 4
GPIO.setup(BUTTON_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
print("Hi...")

while True:
    screen_running = subprocess.run(["screen", "-list", screen_session], capture_output=True, text=True)
    is_screen_running = screen_session in screen_running.stdout
    
    if GPIO.input(BUTTON_PIN) == GPIO.LOW:
        if not is_screen_running:
            subprocess.run(["screen", "-dmS", screen_session, "python3", controller_code])
            print("Button released and screen started")
        else:
            print("Button released but screen already running")
    
    elif GPIO.input(BUTTON_PIN) == GPIO.HIGH:
        if is_screen_running:
            subprocess.run(["screen", "-S", screen_session, "-X", "quit"])
            print("Button released and screen killed")
        else:
            print("Button is pressed..")
    time.sleep(0.1)

GPIO.cleanup()

