import RPi.GPIO as GPIO

BUTTON_PIN = 7  # GPIO physical pin number on the Raspberry Pi. Pin 7 == GPIO4

GPIO.setmode(GPIO.BOARD) # Use the physical pin numbers
GPIO.setup(BUTTON_PIN, GPIO.IN) #Input pin

try:
    while True:
        button_state = GPIO.input(BUTTON_PIN)
        if button_state == GPIO.HIGH:
            print("Detected!")
        else:
            print("-----------------------")

except KeyboardInterrupt:
    pass

finally:
    GPIO.cleanup()