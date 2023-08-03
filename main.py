from class_driver.class_driver import motor_driver
import time
import RPi.GPIO as GPIO 





def round():
    left_driver.raw = float(346)
    left_driver.lll = float(384)
    time.sleep(0.5)
    left_driver.raw = float(303)
    left_driver.lll = float(382)
    time.sleep(0.5)
    left_driver.raw = float(300)
    left_driver.lll = float(327)
    time.sleep(0.5)
    left_driver.raw = float(346)
    left_driver.lll = float(325)
    time.sleep(0.5)

def center():
    # 回到中点
    left_driver.raw = float(323) # 右小 左大
    left_driver.lll = float(359) # 上大 下小



if __name__ == '__main__':
    left_driver = motor_driver("/dev/left_roll",115200)
    
    left_driver.start()
    
    GPIO.setwarnings(False)
    GPIO.setmode(GPIO.BCM)

    GPIO.setup(26,GPIO.IN,pull_up_down=GPIO.PUD_DOWN)
    GPIO.setup(27,GPIO.IN,pull_up_down=GPIO.PUD_DOWN)
    GPIO.setup(21,GPIO.OUT)
    buzze = GPIO.PWM(21,440)
    buzze.start(0)


    try:
        while True:

            if GPIO.input(27):
                center()
                print("center")
                buzze.ChangeDutyCycle(50)
                time.sleep(1)
                buzze.ChangeDutyCycle(0)
            elif GPIO.input(26):
                round()
                print("round")
                buzze.ChangeDutyCycle(50)
                time.sleep(1)
                buzze.ChangeDutyCycle(0)
            else:
                pass
            

    except:
        left_driver.raw = 370
        left_driver.lll = 370
        time.sleep(0.1)
        exit()