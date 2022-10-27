
from numpy import argmin
from math import atan2


# Math
import math
# import schedule
import time
from numpy import nan


class CollisionAvoidance_jy():
    def __init__(self):
        print('CollisionAvoidance_jy')

        self.ObsDist = 0.0
        self.ObsDist2 = 0.0

        self.Target = [0.0, 0.0]

        self.LidarSampling = 0
        self.AvoidancePos = [0.0] * 2
        self.LOSCA = [0.0] * 3
        self.RIGHTLOSCA = [0.0] * 3
        self.LEFTLOSCA = [0.0] * 3
        self.time = 0.0
        self.timedelay_10ms = 0.0
        self.timedelay_50ms = 0.0
        self.waypointtime = 0.0
        self.movingtime = 0.0
        self.ObsAngle = 0.0
        self.altitude = 0.0
        self.ranges = 0.0
        self.alt = 0.0
        self.takeoffflag = True
        self.R2D = math.pi / 180
        self.D2R = 180 / math.pi 
        self.SinAlgo = 0.0
        self.ObsPos = [0.0] * 2
        self.lidarcount = 0.0
        self.offcount = 0.0
        self.imu = 0.0
        self.qt = [0.0] * 4
        self.Heading = 0.0
        self.lat = 0.0
        self.lon = 0.0
        self.alt = 0.0
        self.posX = 5.0
        self.posY = 5.0
        self.posZ = -5.0
        self.bodyX = 0.0
        self.bodyY = 0.0
        self.yawcmd = 0.0
        self.ObsSizeAngle = 0.0
        self.AvoidanceFlag = False
        # Offboard Mode Counter
        self.OffboardCount = 0

        # TrajectorySetpoint
        self.InitialPosition = [0.0, 0.0, -5.0]
        self.SetPosition = [80.0, 0.0, -2.1] 


    # Main Function
    def CA(self, x, y, ObsAngle, ObsSize, AvoidanceFlag):
        self.AvoidanceFlag = AvoidanceFlag
        self.PosX = x
        self.PosY = y
        self.ObsAngle = ObsAngle
        self.ObsSize = ObsSize
        
        vel_cmd_x, vel_cmd_y, vel_cmd_z, vel_cmd_yaw = 0.0, 0.0, 0.0, 0.0
        if self.AvoidanceFlag == True and (336.0 < self.ObsAngle < 360.0 or 0.0 < self.ObsAngle < 66.0): #Front View

            if self.posZ <= -2.0:
                if 14.0 < self.ObsAngle < 194.0:
                    print("LEFT!!!")
                    vel_cmd_x, vel_cmd_y, vel_cmd_z, vel_cmd_yaw = self.Collector()
                else:

                    print("RIGHT!!!")
                    vel_cmd_x, vel_cmd_y, vel_cmd_z, vel_cmd_yaw = self.FrontLosca()     

        elif self.AvoidanceFlag == True and 66.0 <= self.ObsAngle < 156.0:
            if self.posZ <=  -2.0:
                print("Right obstacle avoidance")
                vel_cmd_x, vel_cmd_y, vel_cmd_z, vel_cmd_yaw = self.RightSideLosca()

        elif self.AvoidanceFlag == True and 246.0 <= self.ObsAngle <= 336.0:
            if self.posZ <=  -2.0:
                print("Left obstacle avoidance")
                vel_cmd_x, vel_cmd_y, vel_cmd_z, vel_cmd_yaw = self.LeftSideLosca()

        else:
            print("Keep Going")
            #self.TrajectorySetpointCallback([self.SetPosition[0], self.SetPosition[1], self.SetPosition[2]])
        vel_cmd_yaw = atan2(vel_cmd_x,vel_cmd_y) * 180 / 3.14

        return vel_cmd_x, vel_cmd_y, vel_cmd_z, vel_cmd_yaw 



    def FrontLosca(self):
        vel_cmd_x = self.posX -1.0
        vel_cmd_y = self.posY + self.ObsSize + 1.0
        vel_cmd_z = self.SetPosition[2]
        yaw_cmd = 0.0

        return vel_cmd_x, vel_cmd_y, vel_cmd_z, yaw_cmd
       

    def RightSideLosca(self):
        vel_cmd_x = self.SetPosition[0]
        vel_cmd_y = self.posY - 1.0
        vel_cmd_z = self.SetPosition[2]
        yaw_cmd = 0.0

        return vel_cmd_x, vel_cmd_y, vel_cmd_z, yaw_cmd

    def LeftSideLosca(self):

        vel_cmd_x = self.SetPosition[0]
        vel_cmd_y = self.posY + 1.0
        vel_cmd_z = self.SetPosition[2]
        yaw_cmd = 0.0

        return vel_cmd_x, vel_cmd_y, vel_cmd_z, yaw_cmd

    def Collector(self):
        vel_cmd_x = self.posX -1.0
        vel_cmd_y = self.posY - (self.ObsSize + 1.0)
        vel_cmd_z = self.SetPosition[2]
        yaw_cmd = 0.0

        return vel_cmd_x, vel_cmd_y, vel_cmd_z, yaw_cmd 

        


