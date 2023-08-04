import RPi.GPIO as GPIO
GPIO.setmode(GPIO.BCM)
GPIO.setup(26,GPIO.IN,pull_up_down=GPIO.PUD_DOWN)
GPIO.setup(27,GPIO.IN,pull_up_down=GPIO.PUD_DOWN)
GPIO.setwarnings(False)
while 1:
    if GPIO.input(26) == True:
        print(26)
    elif GPIO.input(27) == True:
        print(27)
    else:
        print(28)