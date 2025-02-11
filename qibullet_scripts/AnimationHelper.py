import time
from qibullet import SimulationManager
from qibullet import PepperVirtual
from itertools import cycle

class AnimationHelper:
    """
    A helper class to manage joint values and apply movements for robot animations.

    Attributes:
        values (dict): Dictionary containing the min, max, and default joint values for different parts of the robot.
    """

    def __init__(self):
        # Initialize the joint values
        self.joint_values = {
            "KneePitch": {"default": 1.2487480867153358e-05, "max": 0.5150751521859811, "min": -0.5150534730360238},
            "HipPitch": {"default": -0.026077747344970717, "max": 1.038470000000001, "min": -1.0385202887120204},
            "HipRoll": {"default": -0.004601949581126251, "max": 0.5148724452819013, "min": -0.514872556331641},
            "HeadYaw": {"default": -0.007669799859313792, "max": 2.08566996723598, "min": -2.085669957095783},
            "HeadPitch": {"default": -0.21171366263390057, "max": 0.6370555077443466, "min": -0.7068831413088167},
            "LShoulderPitch": {"default": 1.5799913325347883, "max": 2.0856327137354382, "min": -2.0857193429951213},
            "LShoulderRoll": {"default": 0.11658105878565315, "max": 1.5620352563587883, "min": 0.046324216034425464},
            "LElbowYaw": {"default": -1.2179857959441898, "max": 2.0856730218757935, "min": -2.0856702705863217},
            "LElbowRoll": {"default": -0.518485033449426, "max": -0.008729231271612178, "min": -1.493277200931256},
            "LWristYaw": {"default": -0.030718155772637284, "max": 1.8238720476387489, "min": -1.823866203498663},
            "LHand": {"default": 0.4490305675799956, "max": 0.7615251132828093, "min": 0.11772619798183623},
            "LFinger21": {"default": 0.5145652653426582, "max": 0.872665710462198, "min": 3.256431240736204e-05},
            "LFinger22": {"default": 0.5145550956015306, "max": 0.8726605754810258, "min": 7.571678298148382e-06},
            "LFinger23": {"default": 0.5145433975924407, "max": 0.8726580945555916, "min": -5.2625408576812825e-06},
            "LFinger11": {"default": 0.514550907369978, "max": 0.8726528493829535, "min": 1.7244542840840105e-05},
            "LFinger12": {"default": 0.514560909303441, "max": 0.8726648366416455, "min": 1.4402824322664258e-05},
            "LFinger13": {"default": 0.5145322453639996, "max": 0.8726469424040322, "min": -4.615652311460902e-06},
            "LFinger41": {"default": 0.5145440255086989, "max": 0.8726459097316875, "min": 1.1627067719074748e-05},
            "LFinger42": {"default": 0.5145713643674276, "max": 0.8726646501623663, "min": 2.7520603940252335e-05},
            "LFinger43": {"default": 0.5145314951973188, "max": 0.8726461922335103, "min": -6.245261650260918e-06},
            "LFinger31": {"default": 0.5145480127441004, "max": 0.8726466038753596, "min": 1.92111150703092e-05},
            "LFinger32": {"default": 0.5145682058742855, "max": 0.8726653066229897, "min": 2.348271978554362e-05},
            "LFinger33": {"default": 0.5145324163033798, "max": 0.8726471133327843, "min": -5.829355579127947e-06},
            "LThumb1": {"default": 0.5145454137092182, "max": 0.8726576356318866, "min": 5.185164763482684e-07},
            "LThumb2": {"default": 0.5145598996083875, "max": 0.8726650000000002, "min": 9.596677038944638e-06},
            "RShoulderPitch": {"default": 1.5799891548771352, "max": 2.085632255274312, "min": -2.085670000000001},
            "RShoulderRoll": {"default": -0.1150475835612839, "max": -0.05147930367196498, "min": -1.562063944817747},
            "RElbowYaw": {"default": 1.2256556824990978, "max": 2.085670000000001, "min": -2.085670000000001},
            "RElbowRoll": {"default": 0.5184863593975079, "max": 1.5040252914645436, "min": 0.008734659432886656},
            "RWristYaw": {"default": 0.027562902505467793, "max": 1.8238629932791943, "min": -1.8238700000071715},
            "RHand": {"default": 0.44836554749656027, "max": 0.7615425787907705, "min": 0.09670029602513969},
            "RFinger41": {"default": 0.5138004216152642, "max": 0.8726650000000002, "min": 3.2500131831598925e-05},
            "RFinger42": {"default": 0.5137876335305431, "max": 0.8726613971093217, "min": 6.621770522881207e-06},
            "RFinger43": {"default": 0.5137858430563587, "max": 0.872665000241991, "min": 2.44721959218808e-06},
            "RFinger31": {"default": 0.513798893193898, "max": 0.8726650000000631, "min": 2.9591900040574207e-05},
            "RFinger32": {"default": 0.5137867856201038, "max": 0.8726611924652498, "min": 5.5007643569382396e-06},
            "RFinger33": {"default": 0.5137793092829542, "max": 0.8726609139910638, "min": -4.44161471392458e-06},
            "RFinger21": {"default": 0.5137894198173238, "max": 0.8726594335954687, "min": 2.0562173534962795e-05},
            "RFinger22": {"default": 0.5137927494664768, "max": 0.872663778239324, "min": 1.3024535752394532e-05},
            "RFinger23": {"default": 0.5137706243991499, "max": 0.872652229011438, "min": -5.184972803359497e-06},
            "RFinger11": {"default": 0.5137888508150772, "max": 0.872662718810156, "min": 1.502372945295803e-05},
            "RFinger12": {"default": 0.5137902854934814, "max": 0.872662103849504, "min": 1.0473786772423725e-05},
            "RFinger13": {"default": 0.5137693331119889, "max": 0.8726509377022722, "min": -7.962126508317546e-13},
            "RThumb1": {"default": 0.5137808468301717, "max": 0.8726597217378669, "min": 2.335433915126734e-06},
            "RThumb2": {"default": 0.5137892987582103, "max": 0.8726650000781971, "min": 5.903319839337164e-06},
        }

    def setting_joint_values(self, group_name: str, values) -> dict:
        """
        Sets joint values for different parts of the robot.

        Parameters:
            - group_name (str): The name of the joint group. Supported group names:
                - "right_shoulder": ["RShoulderRoll", "RShoulderPitch"]
                - "right_elbow": ["RElbowYaw", "RElbowRoll"]
                - "right_fingers": ["RFinger11" to "RFinger43", "RThumb1", "RThumb2"]
                - "right_hand": ["RWristYaw", "RHand"]
                - "left_shoulder": ["LShoulderRoll", "LShoulderPitch"]
                - "left_elbow": ["LElbowYaw", "LElbowRoll"]
                - "left_fingers": ["LFinger11" to "LFinger43", "LThumb1", "LThumb2"]
                - "left_hand": ["LWristYaw", "LHand"]
                - "head": ["HeadYaw", "HeadPitch"]
                - "hip": ["HipPitch", "HipRoll"]
                - "knee": ["KneePitch"]

            - values: Joint values to set for the specified group.
                - For fingers ("right_fingers" and "left_fingers"), `values` can be:
                    - A single value (int or float) for all fingers.
                    - A 2D list of values where each sublist corresponds to a finger group.
                - For other groups (except "knee"), `values` should be a list with two values.
                - For "knee", `values` should be a single value (int or float).

        Returns:
            dict: A dictionary mapping joint names to their respective values.

        Example Usage:
        - setting_joint_values("right_shoulder", [0.5, -0.3])
        - setting_joint_values("right_fingers", 0.8)
        - setting_joint_values("right_fingers", [[0.8, 0.8, 0.8], [0.8, 0.8, 0.8], [0.8, 0.8, 0.8], [0.8, 0.8, 0.8], [0.8, 0.8]])
        - setting_joint_values("knee", 0.2)
        """
        group_names = ["right_shoulder", "right_elbow", "right_fingers", "right_hand", "left_shoulder", 
                       "left_elbow", "left_fingers", "left_hand", "head", "hip", "knee"]

        # Initialize joint_values
        joint_values = {}

        # Right Side
        if group_name == group_names[0]:  # Right Shoulder
            joint_values = {"RShoulderRoll": values[0], "RShoulderPitch": values[1]}
        elif group_name == group_names[1]:  # Right Elbow
            joint_values = {"RElbowYaw": values[0], "RElbowRoll": values[1]}
        elif group_name == group_names[2]:  # Right Fingers
            if isinstance(values, (int, float)):
                joint_values = {f"RFinger{finger}{digit}": values for finger in [1, 2, 3, 4] for digit in range(1, 4)}
                joint_values.update({"RThumb1": values, "RThumb2": values})
            else:
                joint_values = {f"RFinger{finger}{digit}": values[finger - 1][digit - 1] for finger in range(1, 5) for digit in range(1, 4)}
                joint_values.update({"RThumb1": values[4][0], "RThumb2": values[4][1]})
        elif group_name == group_names[3]:  # Right Hand
            joint_values = {"RWristYaw": values[0], "RHand": values[1]}

        # Left Side
        elif group_name == group_names[4]:  # Left Shoulder
            joint_values = {"LShoulderRoll": values[0], "LShoulderPitch": values[1]}
        elif group_name == group_names[5]:  # Left Elbow
            joint_values = {"LElbowYaw": values[0], "LElbowRoll": values[1]}
        elif group_name == group_names[6]:  # Left Fingers
            if isinstance(values, (int, float)):
                joint_values = {f"LFinger{finger}{digit}": values for finger in [1, 2, 3, 4] for digit in range(1, 4)}
                joint_values.update({"LThumb1": values, "LThumb2": values})
            else:
                joint_values = {f"LFinger{finger}{digit}": values[finger - 1][digit - 1] for finger in range(1, 5) for digit in range(1, 4)}
                joint_values.update({"LThumb1": values[4][0], "LThumb2": values[4][1]})
        elif group_name == group_names[7]:  # Left Hand
            joint_values = {"LWristYaw": values[0], "LHand": values[1]}

        # Others
        elif group_name == group_names[8]:  # Head
            joint_values = {"HeadYaw": values[0], "HeadPitch": values[1]}
        elif group_name == group_names[9]:  # Hips
            joint_values = {"HipPitch": values[0], "HipRoll": values[1]}
        elif group_name == group_names[10]:  # Knee
            joint_values = {"KneePitch": values}

        return joint_values

    def applying_joint_values(self, robot, joints_data, speed):
        """
        Applies joint values to the robot.

        Parameters:
            robot: The robot object to control.
            joints_data (dict): Dictionary of joint names and angles to apply.
            speed (float): Speed for the robot to apply the joint movements.
        """
        for joint_name, angle in joints_data.items():
            print(f"Setting {joint_name} to {angle}")
            robot.setAngles(joint_name, angle, speed)

    def default_pose(self, robot, animation_speed = 0.2, rest_time = 2.0):
        robot.goToPosture("Stand", animation_speed)
        time.sleep(rest_time)

    def print_action_title(self, title):
        print("-"*60)
        print(" "*20,f"{title}")
        print("-"*60)
