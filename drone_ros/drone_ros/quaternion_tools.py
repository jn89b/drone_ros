import math  
import numpy as np
from typing import Dict, List, Any

def get_euler_from_quaternion(x:float, y:float, z:float, w:float) -> tuple:
        """
        Convert a quaternion into euler angles (roll, pitch, yaw)
        roll is rotation around x in radians (counterclockwise)
        pitch is rotation around y in radians (counterclockwise)
        yaw is rotation around z in radians (counterclockwise)
        """
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)
     
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)
     
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)
     
        return roll_x, pitch_y, yaw_z # in radians

 
def get_quaternion_from_euler(roll:float, pitch:float, yaw:float) -> list:
  """
  Convert an Euler angle to a quaternion.
   
  Input
    :param roll: The roll (rotation around x-axis) angle in radians.
    :param pitch: The pitch (rotation around y-axis) angle in radians.
    :param yaw: The yaw (rotation around z-axis) angle in radians.
 
  Output
    :return qx, qy, qz, qw: The orientation in quaternion [x,y,z,w] format
  """

  qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
  qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
  qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
  qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)

  return [qx, qy, qz, qw]


def to_quaternion_from_euler_dgs(roll=0.0, pitch=0.0, yaw=0.0):
    """
    Convert degrees to quaternions
    """
    t0 = math.cos(math.radians(yaw * 0.5))
    t1 = math.sin(math.radians(yaw * 0.5))
    t2 = math.cos(math.radians(roll * 0.5))
    t3 = math.sin(math.radians(roll * 0.5))
    t4 = math.cos(math.radians(pitch * 0.5))
    t5 = math.sin(math.radians(pitch * 0.5))

    w = t0 * t2 * t4 + t1 * t3 * t5
    x = t0 * t3 * t4 - t1 * t2 * t5
    y = t0 * t2 * t5 + t1 * t3 * t4
    z = t1 * t2 * t4 - t0 * t3 * t5

    return [w, x, y, z]


#rotation 3d
def rot3d(roll, pitch, yaw):
    R_x = np.array([[1, 0, 0],
                    [0, np.cos(roll), -np.sin(roll)],
                    [0, np.sin(roll), np.cos(roll)]])

    R_y = np.array([[np.cos(pitch), 0, np.sin(pitch)],
                    [0, 1, 0],
                    [-np.sin(pitch), 0, np.cos(pitch)]])

    R_z = np.array([[np.cos(yaw), -np.sin(yaw), 0],
                    [np.sin(yaw), np.cos(yaw), 0],
                    [0, 0, 1]])

    R = np.dot(R_x, np.dot(R_y, R_z))
    return R

def rot2d(psi):
    return np.array([[np.cos(psi), -np.sin(psi)],
                     [np.sin(psi), np.cos(psi)]])


def convertENUToNED(x_enu:float, y_enu:float, z_enu:float) -> list:
    """converts from ENU to NED"""
    ned =  np.zeros(3, dtype=np.float64)
    ned[0] = y_enu
    ned[1] = x_enu
    ned[2] = -z_enu
    return ned

def convertNEDToENU(x_ned:float, y_ned:float, z_ned:float) -> list:
    """converts from NED to ENU"""
    #create 3,1 array
    enu = np.zeros(3, dtype=np.float64)
    enu[0] = y_ned
    enu[1] = x_ned
    enu[2] = -z_ned
    return enu

    
def get_quaternion_from_euler(roll:float, pitch:float, yaw:float) -> list:
  """
  Convert an Euler angle to a quaternion.
   
  Input
    :param roll: The roll (rotation around x-axis) angle in radians.
    :param pitch: The pitch (rotation around y-axis) angle in radians.
    :param yaw: The yaw (rotation around z-axis) angle in radians.
 
  Output
    :return qx, qy, qz, qw: The orientation in quaternion [x,y,z,w] format
  """

  qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
  qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
  qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
  qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)

  return [qx, qy, qz, qw]


def convertENUToNED(x_enu:float, y_enu:float, z_enu:float) -> list:
    """converts from ENU to NED"""
    ned =  np.zeros(3, dtype=np.float64)
    ned[0] = y_enu
    ned[1] = x_enu
    ned[2] = -z_enu
    return ned

def convertNEDToENU(x_ned:float, y_ned:float, z_ned:float) -> list:
    """converts from NED to ENU"""
    #create 3,1 array
    enu = np.zeros(3, dtype=np.float64)
    enu[0] = y_ned
    enu[1] = x_ned
    enu[2] = -z_ned
    return enu

def convertNEDtoENUVector(x_ned:np.ndarray, y_ned:np.ndarray, z_ned:np.ndarray) -> list:
    """converts from NED to ENU"""
    enu_x = y_ned
    enu_y = x_ned
    enu_z = -z_ned
    return enu_x, enu_y, enu_z

def convertENUToNEDVector(x_enu:np.ndarray, y_enu:np.ndarray, z_enu:np.ndarray) -> list:
    """converts from ENU to NED"""
    ned_x = y_enu
    ned_y = x_enu
    ned_z = -z_enu
    return ned_x, ned_y, ned_z  


def ned_to_enu_states(ned_state:np.array) -> np.array:
    enu_state: np.array = np.zeros(7)
    enu_state[0] = ned_state[1] 
    enu_state[1] = ned_state[0] 
    enu_state[2] = -ned_state[2]
    enu_state[3] = -ned_state[3]
    enu_state[4] = ned_state[4]
    enu_state[5] = yaw_ned_to_enu(ned_state[5])
    enu_state[6] = ned_state[6]
    return enu_state

def enu_to_ned_states(enu_state:np.array) -> np.array:
    ned_state: np.array = np.zeros(7)
    ned_state[0] = enu_state[1] 
    ned_state[1] = enu_state[0] 
    ned_state[2] = -enu_state[2]
    ned_state[3] = -enu_state[3]
    ned_state[4] = enu_state[4]
    ned_state[5] = yaw_enu_to_ned(enu_state[5])
    ned_state[6] = enu_state[6]
    return ned_state
    
def enu_to_ned_controls(enu_controls:np.array,
                        change_yaw:bool=False) -> np.array:
    """
    Controls are in the form of [phi, theta, psi, velocity]
    """
    ned_controls: np.array = np.zeros(4)
    ned_controls[0] = -enu_controls[0]
    ned_controls[1] = enu_controls[1]
    if change_yaw:
        ned_controls[2] = -enu_controls[2]
    else:
        ned_controls[2] = enu_controls[2] 
    ned_controls[3] = enu_controls[3]
    return ned_controls
    
def enu_to_ned_controls(enu_controls:np.array,
                        change_yaw:bool=False) -> np.array:
    """
    Controls are in the form of [phi, theta, psi, velocity]
    """
    
    ned_controls: np.array = np.zeros(4)
    ned_controls[0] = -enu_controls[0]
    ned_controls[1] = enu_controls[1]
    if change_yaw:
        ned_controls[2] = -enu_controls[2]
    else:
        ned_controls[2] = enu_controls[2]
    ned_controls[3] = enu_controls[3]
    return ned_controls
    
def wrap_to_pi(angle:float) -> float:
    """
    Wrap an angle in radians to the range [-pi, pi].

    Parameters:
        angle (float): Angle in radians.
    
    Returns:
        float: Angle wrapped to [-pi, pi].
    """
    return (angle + np.pi) % (2 * np.pi) - np.pi

def yaw_ned_to_enu(yaw_ned:float) -> float:
    """
    Convert yaw angle from NED to ENU.
    
    In the NED frame, yaw is measured clockwise from North.
    In the ENU frame, yaw is measured counterclockwise from East.
    
    The conversion formula in radians is:
        yaw_enu = (pi/2 - yaw_ned) wrapped to [-pi, pi]

    Parameters:
        yaw_ned (float): Yaw angle in radians in the NED frame.
        
    Returns:
        float: Yaw angle in radians in the ENU frame.
    """
    yaw_enu = np.pi/2 - yaw_ned
    return wrap_to_pi(yaw_enu)

def yaw_enu_to_ned(yaw_enu:float)-> float:
    """
    Convert yaw angle from ENU to NED.
    
    The conversion is symmetric:
        yaw_ned = (pi/2 - yaw_enu) wrapped to [-pi, pi]

    Parameters:
        yaw_enu (float): Yaw angle in radians in the ENU frame.
        
    Returns:
        float: Yaw angle in radians in the NED frame.
    """
    yaw_ned = np.pi/2 - yaw_enu
    return wrap_to_pi(yaw_ned)


def get_model_yaw_command(ned_yaw_cmd:float,
                               ned_yaw:float) -> float:
    """
    The math model likes the u_psi command to be 
    in the global frame. This function converts 
    ned_yaw_cmd
    """    
    yaw_cmd:float = ned_yaw_cmd + ned_yaw
    
    # wrap the angle to [-pi, pi]
    return wrap_to_pi(yaw_cmd)

def get_relative_ned_yaw_cmd(
        ned_yaw:float, 
        relative_yaw_ned_cmd:float) -> float:

    yaw_cmd:float = relative_yaw_ned_cmd - ned_yaw
        
        # wrap the angle to [-pi, pi]
    return wrap_to_pi(yaw_cmd)


def compute_pursuit_angle(enu_state:np.array,
                          target_x, 
                          target_y) -> float:
    """
    Yaw command is in global frame in ENU convention
    """
    dx:float = target_x - enu_state[0]
    dy:float = target_y - enu_state[1]
    enu_yaw_cmd:float = np.arctan2(dy, dx)

    return enu_yaw_cmd


def convert_enu_state_sol_to_ned(
    states:Dict[str, np.array]) -> Dict[str, np.array]:
    """
    Returns the states in the NED frame 
    """
    ned_x = states['y']
    ned_y = states['x']
    ned_z = -states['z']
    ned_roll = -states['phi']
    ned_pitch = -states['theta']
    ned_yaw = np.pi/2 - states['psi']
    # wrap the yaw to [-pi, pi]
    ned_yaw = wrap_to_pi(ned_yaw)
    ned_v = states['v']
    
    return {
        'x': ned_x,
        'y': ned_y,
        'z': ned_z,
        'phi': ned_roll,
        'theta': ned_pitch,
        'psi': ned_yaw,
        'v': ned_v
    }
    
    
    