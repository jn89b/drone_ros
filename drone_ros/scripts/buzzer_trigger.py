"""
This code will be migrated to pew_pew repository
in the future

User sets a PWM from RC to high 
Then we check if the pin is already set to high and call out the buzzer
Then we check if the pin is already set to low we turn off the buzzer
If pwm is set to middle value then we buzzer is still off 
"""

import gpiod 
import time 

class Buzzer:
    def __init__(self,
                 pin_num:int=3,
                 chip_name:str='../dev/gpiochip4'):
        self.pin_num = pin_num
        self.chip_name = chip_name
        self.chip = gpiod.Chip(self.chip_name)
        self.line = self.chip.get_line(self.pin_num)
        self.line.request(consumer='buzzer', type=gpiod.LINE_REQ_DIR_OUT)
        
    def buzz(self, duration:float=0.5):
        self.line.set_value(1)
        time.sleep(duration)
        self.line.set_value(0)
        
        
if __name__ == '__main__':
    buzzer = Buzzer()
    
    for _ in range(3):
        buzzer.buzz()
        # time.sleep(0.5)
        