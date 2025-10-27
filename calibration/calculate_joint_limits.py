# Converts the calibration data to joint limits in radians for each joint; used in xacro <limit> tags
import json
import math

TPR = (4096/(2*math.pi)) # ticks per radian

def calculate_joint_limits(calibration_file: str)->dict[str, tuple[float, float]]:
    """
    Calculate joint limits from calibration data.

    Args:
        calibration_file (str): Path to the calibration JSON file.
    Returns:
        dict[str, tuple[float, float]]: A dictionary mapping joint names to their (min, max) limits in radians.
    """
    
    with open(calibration_file, 'r') as file:
        calibration_data = json.load(file)

    joint_limits = {}
    for idx, data in calibration_data.items():
        angle_range = (data['range_max'] - data['range_min']) / TPR
        joint_limits[idx] = (round(-angle_range/2, 2), round(angle_range/2, 2))
        
    return joint_limits

if __name__ == "__main__":
    print(calculate_joint_limits('lerobot_calibration.json'))
