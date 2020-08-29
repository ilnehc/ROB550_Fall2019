import time
import numpy as np
import cv2
import IPython

D2R = 3.141592/180.0
R2D = 180.0/3.141592


"""
TODO: Add states and state functions to this class
        to implement all of the required logic for the armlab
"""
class StateMachine():
    def __init__(self, rexarm, planner, kinect):
        self.rexarm = rexarm
        self.tp = planner
        self.kinect = kinect
        self.status_message = "State: Idle"
        self.current_state = "idle"
        self.next_state = "idle"
        self.PointsLearned = []
        
        self.cc = 0
            
    

    def set_next_state(self, state):
        self.next_state = state


    """ This function is run continuously in a thread"""

    def run(self):
        if(self.current_state == "manual"):
            if (self.next_state == "manual"):
                self.manual()
            if(self.next_state == "idle"):
                self.idle()
            if(self.next_state == "estop"):
                self.estop()

        if(self.current_state == "idle"):
            print("crrent state:"+str(self.current_state))
            print("next state : "+str(self.next_state))
            if(self.next_state == "manual"):
                self.manual()
            if(self.next_state == "idle"):
                self.idle()
            if(self.next_state == "estop"):
                self.estop()
            if(self.next_state == "calibrate"):
                self.calibrate()
            if(self.next_state == "execute"):
                self.execute()
            if(self.next_state == "teach"):
                self.teach()
            if(self.next_state=="repeat"):
                self.repeat()
            if(self.next_state=="set_roi"):
                self.set_roi()
            if(self.next_state=="set_exclusion"):
                self.set_exclusion()
            if(self.next_state=="save_frames"):
                self.save_frames()
            if(self.next_state=="task3"):
                self.task3()
            
                
            if(self.next_state=="ClickandGrab"):
                self.ClickandGrab()

        if(self.current_state == "estop"):
            if (self.next_state == "estop"):
                self.estop()
            if (self.next_state == "idle"):
                self.idle()

        if(self.current_state == "calibrate"):
            if(self.next_state == "idle"):
                self.idle()

        if(self.current_state == "execute"):
            if (self.next_state == "execute"):
                self.execute()
            if (self.next_state == "estop"):
                self.estop()
            if (self.next_state == "idle"):
                self.idle()

        if(self.current_state=="teach"):
            if (self.next_state=="teach"):
                self.teach()
            if (self.next_state=="idle"):
                self.idle()

        if (self.current_state=="repeat"):
            if (self.next_state=="repeat"):
                self.repeat()
            if (self.next_state=="idle"):
                self.idle()

        if (self.current_state=="set_roi"):
            if (self.next_state=="set_roi"):
                self.repeat()
            if (self.next_state=="idle"):
                self.idle()

        if (self.current_state=="set_exclusion"):
            if (self.next_state=="set_exclusion"):
                self.set_exclusion()
            if (self.next_state=="idle"):
                self.idle()

        if (self.current_state=="save_frames"):
            if (self.next_state=="save_frames"):
                self.save_frames()
            if (self.next_state=="idle"):
                self.idle()

        if (self.current_state=="task3"):
            if (self.next_state=="task3"):
                self.task3()
            if (self.next_state=="idle"):
                self.idle()

            
               
        if(self.current_state == "ClickandGrab"):
            if (self.next_state == "ClickandGrab"):
                self.ClickandGrab()
            if (self.next_state == "idle"):
                self.idle()



    """Functions run for each state"""


    def manual(self):
        self.status_message = "State: Manual - Use sliders to control arm"
        self.current_state = "manual"
        self.rexarm.send_commands()
        self.rexarm.get_feedback()

    def idle(self):
        self.status_message = "State: Idle - Waiting for input"
        self.current_state = "idle"
        self.rexarm.get_feedback()





    def estop(self):
        self.status_message = "EMERGENCY STOP - Check Rexarm and restart program"
        self.current_state = "estop"
        self.rexarm.disable_torque()
        self.rexarm.get_feedback()

    def calibrate(self):
        self.kinect.kinectCalibrated = False
        self.current_state = "calibrate"
        self.next_state = "idle"
        self.tp.go(max_speed=2.0)
        location_strings = ["lower left corner of board",
                            "upper left corner of board",
                            "upper right corner of board",
                            "lower right corner of board",
                            "center of shoulder motor"]
        i = 0
        for j in range(5):

            self.status_message = "Calibration - Click %s in RGB image" % location_strings[j]
            while (i <= j):

                self.rexarm.get_feedback()

                if(self.kinect.new_click == True):
                    self.kinect.rgb_click_points[i] = self.kinect.last_click.copy()
                    i = i + 1
                    self.kinect.new_click = False

        i = 0
        for j in range(5):
            self.status_message = "Calibration - Click %s in depth image" % location_strings[j]
            while (i <= j):
                self.rexarm.get_feedback()
                if(self.kinect.new_click == True):
                    self.kinect.depth_click_points[i] = self.kinect.last_click.copy()
                    i = i + 1
                    self.kinect.new_click = False


        """TODO Perform camera calibration here"""
        image_points = np.array(self.kinect.rgb_click_points[:])
        depth_points = np.array(self.kinect.depth_click_points[:])
        self.kinect.getAffineTransform(depth_points, image_points)
        self.kinect.kinectCalibrated = True

    def set_roi(self):
        self.current_state = "set_roi"
        self.next_state = "idle"
        self.tp.go(max_speed=2.0)
        location_strings = [ "upper left corner of board",
                            "lower right corner of board"]
        i = 0
        for j in range(2):

            self.status_message = "Setting ROI - Click %s in RGB image" % location_strings[j]
            while (i <= j):

                self.rexarm.get_feedback()

                if(self.kinect.new_click == True):
                    self.kinect.rgb_click_points[i] = self.kinect.last_click.copy()
                    i = i + 1
                    self.kinect.new_click = False

        image_points = np.array(self.kinect.rgb_click_points[:])
        self.kinect.SETROI(image_points)

    def set_exclusion(self):
        self.current_state = "set_exclusion"
        self.next_state = "idle"
        self.tp.go(max_speed=2.0)
        location_strings = [ "upper left corner of board",
                            "lower right corner of board"]
        i = 0
        for j in range(2):

            self.status_message = "Setting exclusion - Click %s in RGB image" % location_strings[j]
            while (i <= j):

                self.rexarm.get_feedback()

                if(self.kinect.new_click == True):
                    self.kinect.rgb_click_points[i] = self.kinect.last_click.copy()
                    i = i + 1
                    self.kinect.new_click = False

        image_points = np.array(self.kinect.rgb_click_points[:])
        self.kinect.SETExclusion(image_points)

    def save_frames(self):
        self.status_message = "State: Saving frames."
        self.current_state = "save_frames"
        self.next_state = "idle"
        self.kinect.SaveBGRandDepthFrames()

    def execute(self):
        self.status_message = "State: execute."
        self.current_state = "execute"
        self.next_state = "idle"
        way_points = np.array([
                                [0.0, 0.0, 0.0, 0.0, 0.0],
                                [1.0, 0.8, 1.0, 0.5, 1.0],
                                [-1.0, -0.8, -1.0, -0.5, -1.0],
                                [-1.0, 0.8, 1.0, 0.5, -1.0],
                                [1.0, -0.8, -1.0, -0.5, -1.0],
                                [0.0, 0.0, 0.0, 0.0, 0.0]
                              ])

        for point in way_points:
            self.rexarm.set_positions(point)
            self.rexarm.pause(2.0)

        """
        p=1
        print("Way points: " + str(way_points[p][:]))
        self.rexarm.set_positions(way_points[p][:])
        """

        print("Execte!")

    def teach(self):
        self.status_message = "State: teaching."
        self.current_state = "teach"
        self.next_state = "idle"
        print('Point learned.', self.rexarm.get_positions())
        temp = self.rexarm.get_positions()
        self.PointsLearned.append(temp[:])


    def repeat(self):
        self.status_message = "State: repeat"
        self.current_state = "repeat"
        self.next_state = "idle"
        nparray = np.array(self.PointsLearned)
        """
        nparray = np.array([
                                [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                                [1.0, 0.8, 1.0, 0.5, 1.0, 0.0],
                                [-1.0, -0.8, -1.0, -0.5, -1.0, 0.0],
                                [-1.0, 0.8, 1.0, 0.5, -1.0, 0.0],
                                [1.0, -0.8, -1.0, -0.5, -1.0, 0.0],
                                [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
                              ])
        """
        print(nparray)
        self.tp.execute_plan(nparray, 2.5)
        #np.savetxt("waypoints.txt",nparray, delimiter="," )
        if (self.rexarm.gripper_state == True):
            self.rexarm.open_gripper()
            self.rexarm.pause(2.0)
            self.rexarm.close_gripper()
        else:
            self.rexarm.close_gripper()
            self.rexarm.pause(2.0)
            self.rexarm.open_gripper()

        """
        for point in nparray:
            self.rexarm.set_positions(point)
            self.rexarm.pause(2.0)
        """
        
    def task3(self):
        self.status_message = "State: task3"
        self.current_state = "task3"
        self.next_state = "idle"
        self.kinect.detectBlocksInDepthImage()
        self.kinect.IdentifyColors()
        #self.kinect.SpecifcColor2Draw = self.kinect.ColorLabel['red']
        self.kinect.SpecifcColor2Draw = self.cc
        self.cc = self.cc + 1
        selector = self.kinect.GetBlockIndexBasedOnColor(self.kinect.SpecifcColor2Draw)
        if selector==-1:
            print('Block color not present.')
        else:
            center, orientation = self.kinect.GetBlockInfo(selector)            
            print('Center: '+str(center))
            print('Orientation: '+str(orientation))

        if self.cc > 7:
            self.cc = 0
       

    def ClickandGrab(self):
        self.current_state = "ClickandGrab"
        self.next_state = "idle"
        valid_pt = False
        self.status_message = "State: Click the block center."
        while(self.kinect.new_click == False) or (valid_pt == False):
            if self.kinect.new_click == False:
                self.rexarm.get_feedback()
            else:
                PointWorld = self.kinect.PointWorld_last_click
                Orientation = self.kinect.BlockOrientation_last_click
                print(Orientation)
                PointWorld = np.asarray(PointWorld)
                PointWorld = PointWorld + np.array([[0.0], [0.0], [10.0]])
                print(PointWorld)
                InnerPointWorld = PointWorld + np.array([[0.0], [0.0], [50.0]])
                #print(InnerPointWorld)
                oren = np.array([0.0, np.pi, np.pi-Orientation])
                WaypointPose = np.hstack((np.transpose(InnerPointWorld)[0], oren))
                Pose = np.hstack((np.transpose(PointWorld)[0], oren))
                print(WaypointPose)
                WaypointAngles = self.rexarm.arm_IK(WaypointPose)
                Angles = self.rexarm.arm_IK(Pose)
                print(np.array(WaypointAngles))
                if (len(Angles[Angles==0])!=24) and (len(WaypointAngles[WaypointAngles==0])!=24):
                    valid_pt = True
                else:
                    self.status_message = "State: Out of workspace! Click the block center again."
                    self.kinect.new_click = False

        self.kinect.new_click = False
        self.rexarm.open_gripper()
        self.tp.execute_point(np.array(WaypointAngles[0]), 5.0)
        self.rexarm.pause(2.0)
        self.tp.execute_point(np.array(Angles[0]), 2.5)
        self.rexarm.pause(2.0)
        self.rexarm.close_gripper()
        self.rexarm.pause(1.5)
        self.tp.execute_point(np.array(WaypointAngles[0]), 2.5)
        self.rexarm.pause(2.0)
        #self.rexarm.set_positions(np.zeros(6))

        self.status_message = "State: Click the drop place."
        valid_pt = False
        while(self.kinect.new_click == False) or (valid_pt == False):
            if self.kinect.new_click == False:
                self.rexarm.get_feedback()
            else:
                PointWorld = self.kinect.PointWorld_last_click
                self.kinect.new_click == False
                PointWorld = np.asarray(PointWorld)
                PointWorld = PointWorld + np.array([[0.0], [0.0], [50.0]])
                InnerPointWorld = PointWorld + np.array([[0.0], [0.0], [50.0]])
                WaypointPose = np.hstack((np.transpose(InnerPointWorld)[0], oren))
                Pose = np.hstack((np.transpose(PointWorld)[0], oren))
                WaypointAngles = self.rexarm.arm_IK(WaypointPose)
                Angles = self.rexarm.arm_IK(Pose)
                if (len(Angles[Angles==0])!=24) and (len(WaypointAngles[WaypointAngles==0])!=24):
                    valid_pt = True
                else:
                    self.status_message = "State: Out of workspace! Click the drop place again."
                    self.kinect.new_click = False
            
        self.kinect.new_click = False
        self.tp.execute_point(np.array(WaypointAngles[0]), 5.0)
        self.tp.execute_point(np.array(Angles[0]), 2.5)
        self.rexarm.open_gripper()
        self.rexarm.pause(1.5)
        self.tp.execute_point(np.array(WaypointAngles[0]), 2.5)
        self.rexarm.set_positions(np.zeros(6))

        #print('Point clicked.', self.rexarm.get_positions())
        #temp = self.rexarm.get_positions()
        #self.PointsLearned.append(temp[:])
