from pymavlink import mavutil

# Create a connection to the vehicle
# For example, connect to a vehicle over a serial port
# Replace '/dev/ttyACM0' with your serial port and '57600' with your baudrate
master = mavutil.mavlink_connection('udp:127.0.0.1:14550')

# Wait for the first heartbeat 
# This confirms the connection to the vehicle
master.wait_heartbeat()

print("Heartbeat from system (system ID: %d component ID: %d)" % (master.target_system, master.target_component))

# Fetch RC channels
def check_rc_channel(channel_num):
    try:
        while True:
            # Read messages
            message = master.recv_match(type='RC_CHANNELS', blocking=True)
            if message:
                # Access the specific RC channel valu
                # Channel values are 1-indexed in MAVLink (channel1_raw is channel 1)
                channel_value = getattr(message, f'chan{channel_num}_raw', None)
                if channel_value is not None:
                    print(f"RC Channel {channel_num} Value: {channel_value}")
                    return channel_value
                else:
                    print(f"RC Channel {channel_num} data not available.")
                    return None
    except Exception as e:
        print(f"An error occurred: {e}")

def get_param_value(param_name):
    try:
        # Fetch the parameter value
        param_value = master.param_fetch_one(param_name)
        print(f"Parameter {param_name} value: {param_value}")
        return param_value
    except Exception as e:
        print(f"An error occurred: {e}")
        return None

if __name__ == '__main__':
    while True:
        #get all parameters
        # print(master.param_fetch_all())
        # Check RC channel 7
        check_rc_channel(7)
        #get_param_value('pew_pew')
