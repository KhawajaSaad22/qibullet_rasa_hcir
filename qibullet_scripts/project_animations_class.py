import time
from qibullet import SimulationManager
from qibullet import PepperVirtual
from itertools import cycle
from AnimationHelper import AnimationHelper

class PepperController:
    def __init__(self):
        self.simulation_manager = SimulationManager()
        self.client = self.simulation_manager.launchSimulation()
        self.robot = self.simulation_manager.spawnPepper(self.client, spawn_ground_plane=True)
        self.helper = AnimationHelper()
        self.helper.default_pose(self.robot)
        
        # Calling the Animation Helper Class
        self.ah = AnimationHelper()
        self.jv = self.ah.joint_values
        self.joint_speed = 0.25

    def perform_animation(self, animation_name):
        if animation_name == "wave":
            self.helper.print_action_title("Waving Right Hand")
            self.waving_right_hand(self.robot)
        elif animation_name == "thumbs_up":
            self.helper.print_action_title("Thumbs Up")
            self.thumbs_up(self.robot)
        elif animation_name == "present_both_hands":
            self.helper.print_action_title("Presenting Both Hands")
            self.present_both_hands(self.robot)
        elif animation_name == "yes":
            self.helper.print_action_title("Yes")
            self.yes(self.robot)
        elif animation_name == "no":
            self.helper.print_action_title("No")
            self.no(self.robot)
        else:
            print(f"Unknown animation: {animation_name}")



    #---------------------------------------------------------------------------------------------------------------
    #                                   Function for Waving Right Hand
    #---------------------------------------------------------------------------------------------------------------

    def waving_right_hand(self, robot, duration = 5.0):
        
        # Defined Values
        # finger_val = 0.872
        finger_val = self.jv["RHand"]["max"]
        speed = 0.25
        
        right_elbow = {"RElbowRoll": [self.jv["RElbowRoll"]["max"], 1.0]} # Right elbow roll in two different states to make to look like wave
        right_shoulder = self.ah.setting_joint_values("right_shoulder", [-1.5, 0.0])
        right_fingers = self.ah.setting_joint_values("right_fingers", finger_val)

        ## Initial setup for the animation    
        self.ah.applying_joint_values(robot, right_shoulder, self.joint_speed)   # for raising the right shoulder 
        self.ah.applying_joint_values(robot, right_fingers, self.joint_speed)    # for raising the right fingers
        
        # Sleep time for completing above setup
        time.sleep(1.0)
        
        # For waving the right elbow with real-time duration check
        start_time = time.time()  # Record the start time
        for joint_name, angles in right_elbow.items():
            joint_names, joint_angles = joint_name, angles

        # Cycle through joint angles until the duration ends
        angle_cycle = cycle(joint_angles)  # Create a cycle iterator for the angles
        while True:
            elapsed_time = time.time() - start_time
            if elapsed_time >= duration:
                print("Time is up, stopping the waving motion.")
                break

            # Get the next angle from the cycle
            angle = next(angle_cycle)
            print(f"joint_name = {joint_names}, angle = {angle}, elapsed_time = {elapsed_time:.2f}s")
            robot.setAngles(joint_names, angle, self.joint_speed)
            time.sleep(0.5)  # Simulate time for the movement


    #---------------------------------------------------------------------------------------------------------------
    #                                   Function for Presenting Right Hand
    #---------------------------------------------------------------------------------------------------------------

    def present_right_hand(self, robot, duration = 3.0):
        
        # Defined Values
        rh = self.jv["RHand"]
        rw = self.jv["RWristYaw"]
        rs = self.jv["RShoulderRoll"]
        re = self.jv["RElbowYaw"]
        rf = self.jv["RFinger11"]

        right_shoulder = self.ah.setting_joint_values("right_shoulder", [rs["default"], 1.0])
        right_elbow = self.ah.setting_joint_values("right_elbow", [re["default"], 1.4])
        right_hand = self.ah.setting_joint_values("right_hand", [rw["max"], rh["min"]+0.3])
        right_fingers = self.ah.setting_joint_values("right_fingers", rf["max"])
        
        ## Initial setup for the animation
        # Some values are manually adjusted for more smooth animation
        self.ah.applying_joint_values(robot, right_shoulder, self.joint_speed-0.15)   # for raising the right shoulder 
        self.ah.applying_joint_values(robot, right_elbow, self.joint_speed-0.15)    # for raising the right elbow
        self.ah.applying_joint_values(robot, right_hand, self.joint_speed-0.1)    # for raising the right hand
        
        # Sleep time for completing above setup
        time.sleep(0.7)

        self.ah.applying_joint_values(robot, right_fingers, 0.1)

        time.sleep(duration)



    #---------------------------------------------------------------------------------------------------------------
    #                                   Function for Presenting Left Hand
    #---------------------------------------------------------------------------------------------------------------

    def present_left_hand(self, robot, duration = 3.0):
        
        # Defined Values
        lh = self.jv["LHand"]
        lw = self.jv["LWristYaw"]
        ls = self.jv["LShoulderRoll"]
        le = self.jv["LElbowYaw"]
        lf = self.jv["LFinger11"]

        left_shoulder = self.ah.setting_joint_values("left_shoulder", [ls["default"], 1.0])
        left_elbow = self.ah.setting_joint_values("left_elbow", [le["default"], -1.4])
        left_hand = self.ah.setting_joint_values("left_hand", [lw["min"], lh["min"]+0.3])
        left_fingers = self.ah.setting_joint_values("left_fingers", lf["max"])
        
        ## Initial setup for the animation
        # Some values are manually adjusted for more smooth animation
        self.ah.applying_joint_values(robot, left_shoulder, self.joint_speed-0.15)   # for raising the left shoulder 
        self.ah.applying_joint_values(robot, left_elbow, self.joint_speed-0.15)    # for raising the left elbow
        self.ah.applying_joint_values(robot, left_hand, self.joint_speed-0.1)    # for raising the left hand
        
        # Sleep time for completing above setup
        time.sleep(0.7)

        self.ah.applying_joint_values(robot, left_fingers, 0.1)

        time.sleep(duration)


    #---------------------------------------------------------------------------------------------------------------
    #                                   Function for Presenting Both Hands
    #---------------------------------------------------------------------------------------------------------------

    def present_both_hands(self, robot, duration = 3.0):
        
        # Defined Values
        rh = self.jv["RHand"]
        rw = self.jv["RWristYaw"]
        rs = self.jv["RShoulderRoll"]
        re = self.jv["RElbowYaw"]
        rf = self.jv["RFinger11"]
        lh = self.jv["LHand"]
        lw = self.jv["LWristYaw"]
        ls = self.jv["LShoulderRoll"]
        le = self.jv["LElbowYaw"]
        lf = self.jv["LFinger11"]

        right_shoulder = self.ah.setting_joint_values("right_shoulder", [rs["default"], 1.0])
        right_elbow = self.ah.setting_joint_values("right_elbow", [re["default"], 1.4])
        right_hand = self.ah.setting_joint_values("right_hand", [rw["max"], rh["min"]+0.3])
        right_fingers = self.ah.setting_joint_values("right_fingers", rf["max"])
        left_shoulder = self.ah.setting_joint_values("left_shoulder", [ls["default"], 1.0])
        left_elbow = self.ah.setting_joint_values("left_elbow", [le["default"], -1.4])
        left_hand = self.ah.setting_joint_values("left_hand", [lw["min"], lh["min"]+0.3])
        left_fingers = self.ah.setting_joint_values("left_fingers", lf["max"])
        
        ## Initial setup for the animation
        # Some values are manually adjusted for more smooth animation
        self.ah.applying_joint_values(robot, right_shoulder, self.joint_speed-0.15)   # for raising the right shoulder 
        self.ah.applying_joint_values(robot, right_elbow, self.joint_speed-0.15)    # for raising the right elbow
        self.ah.applying_joint_values(robot, right_hand, self.joint_speed-0.1)    # for raising the right hand
        self.ah.applying_joint_values(robot, left_shoulder, self.joint_speed-0.15)   # for raising the left shoulder 
        self.ah.applying_joint_values(robot, left_elbow, self.joint_speed-0.15)    # for raising the left elbow
        self.ah.applying_joint_values(robot, left_hand, self.joint_speed-0.1)    # for raising the left hand
        
        # Sleep time for completing above setup
        time.sleep(0.7)

        self.ah.applying_joint_values(robot, right_fingers, 0.1)
        self.ah.applying_joint_values(robot, left_fingers, 0.1)

        time.sleep(duration)


    #---------------------------------------------------------------------------------------------------------------
    #                                   Function for Presenting Right Hand Inverted
    #---------------------------------------------------------------------------------------------------------------

    def present_right_hand_inv(self, robot, duration = 3.0):
        
        # Defined Values
        rh = self.jv["RHand"]
        rw = self.jv["RWristYaw"]
        rs = self.jv["RShoulderRoll"]
        re = self.jv["RElbowYaw"]
        rf = self.jv["RFinger11"]

        right_shoulder = self.ah.setting_joint_values("right_shoulder", [rs["default"], 1.0])
        right_elbow = self.ah.setting_joint_values("right_elbow", [re["default"], 1.4])
        right_hand = self.ah.setting_joint_values("right_hand", [rw["min"]+0.4, rh["min"]+0.3])
        right_fingers = self.ah.setting_joint_values("right_fingers", rf["max"])
        
        ## Initial setup for the animation
        # Some values are manually adjusted for more smooth animation
        self.ah.applying_joint_values(robot, right_shoulder, self.joint_speed-0.15)   # for raising the right shoulder 
        self.ah.applying_joint_values(robot, right_elbow, self.joint_speed-0.15)    # for raising the right elbow
        self.ah.applying_joint_values(robot, right_hand, self.joint_speed-0.1)    # for raising the right hand
        
        # Sleep time for completing above setup
        time.sleep(0.7)

        self.ah.applying_joint_values(robot, right_fingers, 0.1)

        time.sleep(duration)



    #---------------------------------------------------------------------------------------------------------------
    #                                   Function for Presenting Left Hand Inverted
    #---------------------------------------------------------------------------------------------------------------

    def present_left_hand_inv(self, robot, duration = 3.0):
        
        # Defined Values
        lh = self.jv["LHand"]
        lw = self.jv["LWristYaw"]
        ls = self.jv["LShoulderRoll"]
        le = self.jv["LElbowYaw"]
        lf = self.jv["LFinger11"]

        left_shoulder = self.ah.setting_joint_values("left_shoulder", [ls["default"], 1.0])
        left_elbow = self.ah.setting_joint_values("left_elbow", [le["default"], -1.4])
        left_hand = self.ah.setting_joint_values("left_hand", [lw["max"]-0.4, lh["min"]+0.3])
        left_fingers = self.ah.setting_joint_values("left_fingers", lf["max"])
        
        ## Initial setup for the animation
        # Some values are manually adjusted for more smooth animation
        self.ah.applying_joint_values(robot, left_shoulder, self.joint_speed-0.15)   # for raising the right shoulder 
        self.ah.applying_joint_values(robot, left_elbow, self.joint_speed-0.15)    # for raising the right elbow
        self.ah.applying_joint_values(robot, left_hand, self.joint_speed-0.1)    # for raising the right hand
        
        # Sleep time for completing above setup
        time.sleep(0.7)

        self.ah.applying_joint_values(robot, left_fingers, 0.1)

        time.sleep(duration)



    #---------------------------------------------------------------------------------------------------------------
    #                                   Function for Counting Fingers Right Hand
    #---------------------------------------------------------------------------------------------------------------

    def counting_right_hand(self, robot, number_of_fingers = 5, duration = 10.0):
        
        # Defined Values
        rh = self.jv["RHand"]
        rw = self.jv["RWristYaw"]
        rs = self.jv["RShoulderRoll"]
        rer = self.jv["RElbowRoll"]
        rf = self.jv["RFinger11"]
        rfm = rf["max"]
        rfl = rf["min"]
        f_val1 = [[rfm, rfm, rfm], [rfl, rfl, rfl],
                [rfl, rfl, rfl], [rfl, rfl, rfl],
                [rfl, rfl]]
        f_val2 = [[rfm, rfm, rfm], [rfm, rfm, rfm],
                [rfl, rfl, rfl], [rfl, rfl, rfl],
                [rfl, rfl]]
        f_val3 = [[rfm, rfm, rfm], [rfm, rfm, rfm],
                [rfm, rfm, rfm], [rfl, rfl, rfl],
                [rfl, rfl]]
        f_val4 = [[rfm, rfm, rfm], [rfm, rfm, rfm],
                [rfm, rfm, rfm], [rfm, rfm, rfm],
                [rfl, rfl]]
        f_val5 = [[rfm, rfm, rfm], [rfm, rfm, rfm],
                [rfm, rfm, rfm], [rfm, rfm, rfm],
                [rfm, rfm]]

        right_shoulder = self.ah.setting_joint_values("right_shoulder", [-0.5, 0.6])
        right_elbow = self.ah.setting_joint_values("right_elbow", [1.9, rer["max"]])
        right_hand = self.ah.setting_joint_values("right_hand", [-1.0, rh["min"]])
        right_fingers = [self.ah.setting_joint_values("right_fingers", f_val1),
                        self.ah.setting_joint_values("right_fingers", f_val2),
                        self.ah.setting_joint_values("right_fingers", f_val3),
                        self.ah.setting_joint_values("right_fingers", f_val4),
                        self.ah.setting_joint_values("right_fingers", f_val5)]
        
        ## Initial setup for the animation
        # Some values are manually adjusted for more smooth animation
        self.ah.applying_joint_values(robot, right_shoulder, self.joint_speed)   # for raising the right shoulder 
        self.ah.applying_joint_values(robot, right_elbow, self.joint_speed)    # for raising the right elbow
        self.ah.applying_joint_values(robot, right_hand, self.joint_speed)    # for raising the right hand
        
        # Sleep time for completing above setup
        time.sleep(0.7)

        # Handling duration
        if isinstance(duration, (int, float)):
            # Single value provided, divide equally
            interval_durations = [duration / number_of_fingers] * number_of_fingers
        elif isinstance(duration, list) and len(duration) == number_of_fingers:
            # List of durations provided
            interval_durations = duration
        else:
            raise ValueError("Duration must be a single float or a list of length equal to the number of fingers.")

        # Counting loop for left hand
        for i in range(number_of_fingers):
            self.ah.applying_joint_values(robot, right_fingers[i], self.joint_speed)
            time.sleep(interval_durations[i])


    #---------------------------------------------------------------------------------------------------------------
    #                                   Function for Counting Fingers Left Hand
    #---------------------------------------------------------------------------------------------------------------

    def counting_left_hand(self, robot, number_of_fingers = 5, duration = 10.0):
        
        # Defined Values
        lh = self.jv["LHand"]
        lw = self.jv["LWristYaw"]
        ls = self.jv["LShoulderRoll"]
        ler = self.jv["LElbowRoll"]
        lf = self.jv["LFinger11"]
        lfm = lf["max"]
        lfl = lf["min"]
        f_val1 = [[lfm, lfm, lfm], [lfl, lfl, lfl],
                [lfl, lfl, lfl], [lfl, lfl, lfl],
                [lfl, lfl]]
        f_val2 = [[lfm, lfm, lfm], [lfm, lfm, lfm],
                [lfl, lfl, lfl], [lfl, lfl, lfl],
                [lfl, lfl]]
        f_val3 = [[lfm, lfm, lfm], [lfm, lfm, lfm],
                [lfm, lfm, lfm], [lfl, lfl, lfl],
                [lfl, lfl]]
        f_val4 = [[lfm, lfm, lfm], [lfm, lfm, lfm],
                [lfm, lfm, lfm], [lfm, lfm, lfm],
                [lfl, lfl]]
        f_val5 = [[lfm, lfm, lfm], [lfm, lfm, lfm],
                [lfm, lfm, lfm], [lfm, lfm, lfm],
                [lfm, lfm]]

        left_shoulder = self.ah.setting_joint_values("left_shoulder", [0.5, 0.6])
        left_elbow = self.ah.setting_joint_values("left_elbow", [-1.9, ler["min"]])
        left_hand = self.ah.setting_joint_values("left_hand", [1.0, lh["min"]])
        left_fingers = [self.ah.setting_joint_values("left_fingers", f_val1),
                        self.ah.setting_joint_values("left_fingers", f_val2),
                        self.ah.setting_joint_values("left_fingers", f_val3),
                        self.ah.setting_joint_values("left_fingers", f_val4),
                        self.ah.setting_joint_values("left_fingers", f_val5)]
        
        ## Initial setup for the animation
        # Some values are manually adjusted for more smooth animation
        self.ah.applying_joint_values(robot, left_shoulder, self.joint_speed)   # for raising the left shoulder 
        self.ah.applying_joint_values(robot, left_elbow, self.joint_speed)    # for raising the left elbow
        self.ah.applying_joint_values(robot, left_hand, self.joint_speed)    # for raising the left hand
        
        # Sleep time for completing above setup
        time.sleep(0.7)

        # Handling duration
        if isinstance(duration, (int, float)):
            # Single value provided, divide equally
            interval_durations = [duration / number_of_fingers] * number_of_fingers
        elif isinstance(duration, list) and len(duration) == number_of_fingers:
            # List of durations provided
            interval_durations = duration
        else:
            raise ValueError("Duration must be a single float or a list of length equal to the number of fingers.")

        # Counting loop for left hand
        for i in range(number_of_fingers):
            self.ah.applying_joint_values(robot, left_fingers[i], self.joint_speed)
            time.sleep(interval_durations[i])



    #---------------------------------------------------------------------------------------------------------------
    #                                   Function for Counting Fingers Both Hand
    #---------------------------------------------------------------------------------------------------------------

    def counting_both_hands(self, robot, number_of_fingers = 10, duration = 10.0):
        
        # Defined Values
        rh = self.jv["RHand"]
        rw = self.jv["RWristYaw"]
        rs = self.jv["RShoulderRoll"]
        rer = self.jv["RElbowRoll"]
        rf = self.jv["RFinger11"]

        lh = self.jv["LHand"]
        lw = self.jv["LWristYaw"]
        ls = self.jv["LShoulderRoll"]
        ler = self.jv["LElbowRoll"]
        lf = self.jv["LFinger11"]
        lfm = lf["max"]
        lfl = lf["min"]
        f_val1 = [[lfm, lfm, lfm], [lfl, lfl, lfl],
                [lfl, lfl, lfl], [lfl, lfl, lfl],
                [lfl, lfl]]
        f_val2 = [[lfm, lfm, lfm], [lfm, lfm, lfm],
                [lfl, lfl, lfl], [lfl, lfl, lfl],
                [lfl, lfl]]
        f_val3 = [[lfm, lfm, lfm], [lfm, lfm, lfm],
                [lfm, lfm, lfm], [lfl, lfl, lfl],
                [lfl, lfl]]
        f_val4 = [[lfm, lfm, lfm], [lfm, lfm, lfm],
                [lfm, lfm, lfm], [lfm, lfm, lfm],
                [lfl, lfl]]
        f_val5 = [[lfm, lfm, lfm], [lfm, lfm, lfm],
                [lfm, lfm, lfm], [lfm, lfm, lfm],
                [lfm, lfm]]


        right_shoulder = self.ah.setting_joint_values("right_shoulder", [-0.5, 0.6])
        right_elbow = self.ah.setting_joint_values("right_elbow", [1.9, rer["max"]])
        right_hand = self.ah.setting_joint_values("right_hand", [-1.0, rh["min"]])
        right_fingers = [self.ah.setting_joint_values("right_fingers", f_val1),
                        self.ah.setting_joint_values("right_fingers", f_val2),
                        self.ah.setting_joint_values("right_fingers", f_val3),
                        self.ah.setting_joint_values("right_fingers", f_val4),
                        self.ah.setting_joint_values("right_fingers", f_val5)]
        left_shoulder = self.ah.setting_joint_values("left_shoulder", [0.5, 0.6])
        left_elbow = self.ah.setting_joint_values("left_elbow", [-1.9, ler["min"]])
        left_hand = self.ah.setting_joint_values("left_hand", [1.0, lh["min"]])
        left_fingers = [self.ah.setting_joint_values("left_fingers", f_val1),
                        self.ah.setting_joint_values("left_fingers", f_val2),
                        self.ah.setting_joint_values("left_fingers", f_val3),
                        self.ah.setting_joint_values("left_fingers", f_val4),
                        self.ah.setting_joint_values("left_fingers", f_val5)]
        
        ## Initial setup for the animation
        # Some values are manually adjusted for more smooth animation
        self.ah.applying_joint_values(robot, right_shoulder, self.joint_speed)   # for raising the right shoulder 
        self.ah.applying_joint_values(robot, right_elbow, self.joint_speed)    # for raising the right elbow
        self.ah.applying_joint_values(robot, right_hand, self.joint_speed)    # for raising the right hand
        self.ah.applying_joint_values(robot, left_shoulder, self.joint_speed)   # for raising the left shoulder 
        self.ah.applying_joint_values(robot, left_elbow, self.joint_speed)    # for raising the left elbow
        self.ah.applying_joint_values(robot, left_hand, self.joint_speed)    # for raising the left hand
        
        # Handling duration
        if isinstance(duration, (int, float)):
            # Single value provided, divide equally
            interval_durations = [duration / number_of_fingers] * number_of_fingers
        elif isinstance(duration, list) and len(duration) == number_of_fingers:
            # List of durations provided
            interval_durations = duration
        else:
            raise ValueError("Duration must be a single float or a list of length equal to the number of fingers.")

        # Right hand counting
        for i in range(min(5, number_of_fingers)):
            self.ah.applying_joint_values(robot, right_fingers[i], self.joint_speed)
            time.sleep(interval_durations[i])

        # Left hand counting
        for i in range(number_of_fingers - 5):
            self.ah.applying_joint_values(robot, left_fingers[i], self.joint_speed)
            time.sleep(interval_durations[5 + i])


    #---------------------------------------------------------------------------------------------------------------
    #                                   Function for Yes
    #---------------------------------------------------------------------------------------------------------------

    def yes(self, robot, duration = 3.0):
        nod_values = [0.5, -0.3]
        head_pitch = "HeadPitch"
        # For waving the right elbow with real-time duration check
        start_time = time.time()  # Record the start time

        # Cycle through joint angles until the duration ends
        angle_cycle = cycle(nod_values)  # Create a cycle iterator for the angles
        while True:
            elapsed_time = time.time() - start_time
            if elapsed_time >= duration:
                print("Time is up, stopping the waving motion.")
                break

            # Get the next angle from the cycle
            angle = next(angle_cycle)
            print(f"joint_name = {head_pitch}, angle = {angle}, elapsed_time = {elapsed_time:.2f}s")
            robot.setAngles(head_pitch, angle, self.joint_speed)
            time.sleep(0.5)  # Simulate time for the movement



    #---------------------------------------------------------------------------------------------------------------
    #                                   Function for No
    #---------------------------------------------------------------------------------------------------------------

    def no(self, robot, duration = 3.0):
        nod_values = [0.7, -0.7]
        head_yaw = "HeadYaw"
        # For waving the right elbow with real-time duration check
        start_time = time.time()  # Record the start time

        # Cycle through joint angles until the duration ends
        angle_cycle = cycle(nod_values)  # Create a cycle iterator for the angles
        while True:
            elapsed_time = time.time() - start_time
            if elapsed_time >= duration:
                print("Time is up, stopping the waving motion.")
                break

            # Get the next angle from the cycle
            angle = next(angle_cycle)
            print(f"joint_name = {head_yaw}, angle = {angle}, elapsed_time = {elapsed_time:.2f}s")
            robot.setAngles(head_yaw, angle, self.joint_speed)
            time.sleep(0.5)  # Simulate time for the movement



    #---------------------------------------------------------------------------------------------------------------
    #                                   Function for No Idea
    #---------------------------------------------------------------------------------------------------------------

    def no_idea(self, robot, duration = 3.0):
        
        self.present_both_hands(robot, 0.0)
        time.sleep(0.5)

        self.no(robot, duration)


    #---------------------------------------------------------------------------------------------------------------
    #                                   Function for Thumbs Up
    #---------------------------------------------------------------------------------------------------------------

    def thumbs_up(self, robot, duration = 3.0):
        
        # Defined Values
        rh = self.jv["RHand"]
        rw = self.jv["RWristYaw"]
        rs = self.jv["RShoulderRoll"]
        re = self.jv["RElbowYaw"]
        rf = self.jv["RFinger11"]
        rfm = rf["max"]
        rfl = rf["min"]
        f_val1 = [[rfl, rfl, rfl], [rfl, rfl, rfl],
                [rfl, rfl, rfl], [rfl, rfl, rfl],
                [rfm, rfm]]
        

        right_shoulder = self.ah.setting_joint_values("right_shoulder", [-0.3735, 0.63])
        right_elbow = self.ah.setting_joint_values("right_elbow", [0.5, 1.5])
        right_hand = self.ah.setting_joint_values("right_hand", [1.5, rh["default"]])
        right_fingers = self.ah.setting_joint_values("right_fingers", f_val1)

        self.ah.applying_joint_values(robot, right_shoulder, self.joint_speed)
        self.ah.applying_joint_values(robot, right_elbow, self.joint_speed)
        self.ah.applying_joint_values(robot, right_hand, self.joint_speed-0.1)
        time.sleep(0.5)
        self.ah.applying_joint_values(robot, right_fingers, self.joint_speed)

        time.sleep(duration)


    #---------------------------------------------------------------------------------------------------------------
    #                                   Function for Manual Adjustment of Joints
    #---------------------------------------------------------------------------------------------------------------

    # Function to adjust angles dynamically and verify positions without restarting the script again and again
    def adjust_angle(self, robot, key_list):
        # Loop to allow multiple adjustments
        while True:
            key = input(f"Enter the name of the key (or type 'exit' to stop): ")
            if key.lower() == 'exit':
                break
            if key not in key_list:
                print(f"Invalid key name. Please select a valid key from: {key_list}")
                continue

            current_angle = robot.getAnglesPosition(key)
            print(f"Current angle of {key} is: {current_angle}")

            try:
                new_angle = float(input(f"Enter the new angle for {key} (between -3.14 to 3.14): "))
                if -3.14 <= new_angle <= 3.14:
                    robot.setAngles(key, new_angle, 0.5)
                    print(f"Set new angle {new_angle} for {key} with speed 0.5")
                    time.sleep(2.0)
                else:
                    print("Invalid angle value. Please enter a value between -3.14 and 3.14.")
            except ValueError:
                print("Please enter a valid numerical value.")
        

    #---------------------------------------------------------------------------------------------------------------
    #---------------------------------------------------------------------------------------------------------------
    #---------------------------------------------------------------------------------------------------------------

    # # Main simulation script
    # def main(self):
    #     # Set up the simulation environment
    #     simulation_manager = SimulationManager()
    #     client = simulation_manager.launchSimulation()

    #     # Spawn a virtual Pepper robot
    #     pepper = simulation_manager.spawnPepper(client, spawn_ground_plane=True)
        
    #     # Set Pepper to an initial position
    #     self.ah.default_pose(pepper, 1.0, 1.0)

    #     # waving_right_hand(pepper)

    #     # List of some of the va`lid keys for joints (used for testing pepper's movement on the runtime)
    #     # Got these values after printing "key" in line 112 from examples 
    #     # (https://github.com/softbankrobotics-research/qibullet/blob/master/examples/pepper_joints_error.py#L112C9-L112C12)
    #     key_list = ["KneePitch", "HipPitch", "HipRoll", "HeadYaw", "HeadPitch", "LShoulderPitch", "LShoulderRoll", "LElbowYaw",
    #                 "LElbowRoll", "LWristYaw", "LHand", "LFinger21", "LFinger22", "LFinger23",
    #                 "LFinger11", "LFinger12", "LFinger13", "LFinger41", "LFinger42", "LFinger43",
    #                 "LFinger31", "LFinger32", "LFinger33", "LThumb1", "LThumb2", "RShoulderPitch",
    #                 "RShoulderRoll", "RElbowYaw", "RElbowRoll", "RWristYaw", "RHand", "RFinger41",
    #                 "RFinger42", "RFinger43", "RFinger31", "RFinger32", "RFinger33", "RFinger21",
    #                 "RFinger22", "RFinger23", "RFinger11", "RFinger12", "RFinger13", "RThumb1",
    #                 "RThumb2"]
        
    #     # Allow dynamic angle adjustment (Uncomment below line to run manual value setting on runtime for joints)
    #     # pepper.move(0.0, 0.0, 0.5)
    #     # adjust_angle(pepper, key_list)
        
    #     self.ah.print_action_title("Waving Right Hand")
    #     self.waving_right_hand(pepper)
    #     self.ah.print_action_title("Presenting Right Hand")
    #     self.present_right_hand(pepper)
    #     self.ah.print_action_title("Presenting Left Hand")
    #     self.present_left_hand(pepper)
    #     self.ah.print_action_title("Default Pose")
    #     self.ah.default_pose(pepper)
    #     self.ah.print_action_title("Presenting Inverse Right Hand")
    #     self.present_right_hand_inv(pepper)
    #     self.ah.print_action_title("Presenting Inverse Left Hand")
    #     self.present_left_hand_inv(pepper)
    #     self.ah.print_action_title("Default Pose")
    #     self.ah.default_pose(pepper)
    #     self.ah.print_action_title("Presenting Both Hands")
    #     self.present_both_hands(pepper)
    #     self.ah.print_action_title("Counting Right Hand")
    #     self.counting_right_hand(pepper, 4, 5.0)
    #     self.ah.print_action_title("Default Pose")
    #     self.ah.default_pose(pepper)
    #     self.ah.print_action_title("Counting Left Hand")
    #     self.counting_left_hand(pepper, 3, 5.0)
    #     self.ah.print_action_title("Nodding Yes")
    #     self.yes(pepper, 3)
    #     self.ah.print_action_title("Default Pose")
    #     self.ah.default_pose(pepper)
    #     self.ah.print_action_title("Nodding No")
    #     self.no(pepper, 3)
    #     self.ah.print_action_title("Default Pose")
    #     self.ah.default_pose(pepper)
    #     self.ah.print_action_title("No Idea/Clue Expression")
    #     self.no_idea(pepper)
    #     self.ah.print_action_title("Default Pose")
    #     self.ah.default_pose(pepper)
    #     self.ah.print_action_title("Thumbs up")
    #     self.thumbs_up(pepper)
    #     self.ah.print_action_title("Default Pose")
    #     self.ah.default_pose(pepper)
    #     self.ah.print_action_title("Counting till 7")
    #     self.counting_both_hands(pepper, 7, 10)
    #     self.ah.print_action_title("Default Pose")
    #     self.ah.default_pose(pepper)
    #     self.ah.print_action_title("Counting till 7")
    #     self.counting_both_hands(pepper, 7, [2, 5, 1, 6, 4, 1, 2])
    #     self.ah.print_action_title("End of Poses")

    #     # Keep the simulation running
    #     input("Press Enter to end the simulation...")

    #     # Stop the simulation when done
    #     simulation_manager.stopSimulation(client)

    # if __name__ == "__main__":
    #     main()
