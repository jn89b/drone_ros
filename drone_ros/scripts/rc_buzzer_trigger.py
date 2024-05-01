"""
This should be moved to the pew pew trigger
Medium is 1470
Low is 970
High is 1975
"""
import gpiod 
import time 
from pymavlink import mavutil

class Buzzer:
    def __init__(self,
                 pin_num:int=3,
                 chip_name:str='../dev/gpiochip4'):
        self.pin_num = pin_num
        self.chip_name = chip_name
        self.chip = gpiod.Chip(self.chip_name)
        self.line = self.chip.get_line(self.pin_num)
        self.line.request(consumer='buzzer', type=gpiod.LINE_REQ_DIR_OUT)
        self.buzzer_val = self.line.get_value()
        
    def set_val(self, val:int, duration:float=0.5):
        # if self.buzzer_val == val:
        #     print("Buzzer already set to this value")
        #     return
        self.line.set_value(val)
        self.buzzer_val = val
        print("buzzer val is set to ", val)

    def release(self):
        self.line.release()

def check_rc_channel(master, channel_num:int) -> int:
    """
    Check the RC channel value
    """
    message = master.recv_match(type='RC_CHANNELS', blocking=True)
    if message:
        # Access the specific RC channel valu
        # Channel values are 1-indexed in MAVLink (channel1_raw is channel 1)
        channel_value = getattr(message, f'chan{channel_num}_raw', None)
        if channel_value is not None:
            return channel_value
        else:
            return None

if __name__ == '__main__':
    buzzer_master = mavutil.mavlink_connection('192.168.135.72:14550')
    #check if heartbeat is received
    buzzer_master.wait_heartbeat()
    print("Heartbeat from system (system %u component %u)" % (buzzer_master.target_system, buzzer_master.target_component))
    rc_channel = 7
    buzzer = Buzzer()
    
    try:
        while True:
            channel_val = check_rc_channel(buzzer_master, rc_channel)
            if channel_val >= 1800:
                #print("setting buzzer to 1")
                buzzer.set_val(1)
            else:
                #print("setting buzzer to 0")
                buzzer.set_val(0)
                
    except KeyboardInterrupt:
        #end buzzer
        buzzer.set_val(0)
        buzzer.release()
        
    finally:
        buzzer.set_val(0)
        
            
    
        