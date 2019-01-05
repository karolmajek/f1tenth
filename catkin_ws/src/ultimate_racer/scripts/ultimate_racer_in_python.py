#!/usr/bin/env python3.4

import numpy as np
import time
import math
import rospy
import sys
from std_msgs.msg import UInt16
from std_msgs.msg import Float32
from sensor_msgs.msg import LaserScan


class move_auto:
    def __init__(self,min_speed,max_speed,starting_esc):

        self.lasteStopMsgTs=time.time()

        self.speed_record=0
        self.eStop=False
        self.eStart=False
        self.eGo=False
        self.esc_brake=1000
        self.min_speed = float(min_speed)
        self.max_speed = float(max_speed)
        self.starting_esc= int(starting_esc)
        self.esc_min = 1550
        self.throttle = 1500

        self.neutral_yaw=1585
        self.yaw = self.neutral_yaw
        self.yaw_range=1200
        self.max_yaw=self.neutral_yaw+self.yaw_range/2
        self.min_yaw=self.neutral_yaw-self.yaw_range/2

        self.safe_obstacle_distance1=0.5
        self.safe_obstacle_distance2=0.17
        self.safe_obstacle_distance3=0.3
        self.non_cont_dist=0.2

        self.kp=1.8 #1.8
        self.kd=6.0 #0.9
        self.prev_error=0.0

        self.fast_drive_distance=6.0
        self.margin_drive_fast=0.5
        self.margin_drive_slow=0.3

        self.slow_speed_chk_points=100

        self.curr_speed=0

        self.angle_increment=math.radians(0.25) #0.004363323096185923

        self.sin_alfa=np.zeros(360*4)

        for i in range(360*4):
            self.sin_alfa[i]=math.sin(math.radians((i+1)*0.25))

        self.pub_esc = rospy.Publisher('/esc', UInt16,
                                   queue_size=1)

        self.pub_servo = rospy.Publisher('/servo', UInt16,
                                   queue_size=1)



        self.sub_spd = rospy.Subscriber("/spd", Float32, self.spdCallback,
                                    queue_size=1)

        self.sub = rospy.Subscriber("/scan", LaserScan, self.laserCallback,
                                    queue_size=1)

        self.sub_eStop = rospy.Subscriber("/eStop", UInt16, self.eStopCallback,
                                    queue_size=1)

    def spdCallback(self, data):
        self.curr_speed=data.data


    def checkIfReachable(self,r1,r2,alfa,margin):
        if (r1-self.safe_obstacle_distance1<r2 and r1>self.safe_obstacle_distance1):

            return True
        else:

            return (r2*self.sin_alfa[alfa]>margin)


    def steerMAX(self,scan,width_margin):


        segs=[]
        #segs.append(0)
        segs.append(180)
        segs.append(len(scan)-180)
        #segs.append(len(scan)-1)


        for i in range(1,len(scan)):
            if (abs(scan[i]-scan[i-1])> self.non_cont_dist):
                segs.append(i)
                segs.append(i-1)


        minRange=min(scan)

        scan2=np.copy(scan)

        idx=0

        isReachable = False

        scan2[:100]=-1
        scan2[-100:]=-1

        while (not isReachable and len(scan2[scan2>0])>0 and minRange>self.safe_obstacle_distance2):
            idx=np.argmax(scan2)

            for s in segs:
                if (s != idx):

                    if not (self.checkIfReachable(scan[idx],scan[s],abs(s-idx),width_margin)):
                        #if (idx==780):
                        #    print(idx,s)
                        #scan2[idx]=-1
                        scan2[max(0,idx-5):min(idx+5,1080)]=-1
                        break
            if (scan2[idx] != -1):
                isReachable=True

        if (isReachable==False):
            yaw=-1

        else:
            yaw=idx

        return yaw


    def eStopCallback(self, data):
        self.lasteStopMsgTs=time.time()
        if (data.data==0):
            self.exec_eStop()
            # print("Emergency Stop!")
        elif (data.data==1):
            self.eStop=False
            # print("Reset!")
        elif (data.data==2309):
            self.eStart=True
            # print("GO!")

    def exec_eStop(self):

        self.eStop=True
        self.yaw=self.neutral_yaw
        self.throttle=self.esc_brake
        self.pub_esc.publish(self.throttle)
        self.pub_servo.publish(self.yaw)


    def speedControl(self,scan,idx,nitro):
        if (nitro==True):
            speed=scan[idx]
        else:
            yl=max(idx-self.slow_speed_chk_points,0)
            yr=min(len(scan),idx+self.slow_speed_chk_points)
            speed=min(scan[yl:yr])*1.5
        return speed

    def speedPID(self,desired_speed):


        if (desired_speed>self.max_speed):
            dspeed=self.max_speed
        elif (desired_speed < self.min_speed):
            dspeed=self.min_speed
        else:
            dspeed=desired_speed

        pid_error = dspeed - self.curr_speed

        error = pid_error * self.kp
        errordot = self.kd * (pid_error - self.prev_error)

        speed_delta=int(error+errordot)

        self.prev_error = pid_error

        if (self.throttle==self.esc_brake and speed_delta>0):
            self.throttle=self.esc_min


        self.throttle+=speed_delta

        if (self.throttle > 2000):
            self.throttle=2000
        if (self.throttle < 1000):
            self.throttle=1000


    def laserCallback(self, data):


        ts=time.time()

        if(time.time()-self.lasteStopMsgTs>0.5 and (self.eGo or self.eStart)):
            self.exec_eStop()


        mid_point=int(len(data.ranges)/2)

        if (min(data.ranges[mid_point-20:mid_point+20]) < self.safe_obstacle_distance3):
            self.throttle=self.esc_brake
            self.pub_esc.publish(self.throttle)

        scan= np.array(data.ranges)
        scan[scan>10]=10
        #scan[scan<0]=0
        scan[scan<0.06]=0.06
        scan[:90]=10.0
        scan[-90:]=10.0



        turbo=False

        idx=self.steerMAX(scan,self.margin_drive_fast)

        #print(idx, scan[idx])


        if (idx==-1 or scan[idx]<self.fast_drive_distance):
            idx=self.steerMAX(scan,self.margin_drive_slow)
            if (idx==-1):
                # print('not reachable',min(scan),np.argmin(scan))
                self.throttle=self.esc_brake
                desired_speed=0
            else:
                #pass
                desired_speed=self.speedControl(scan,idx,turbo) #self.esc_min
        else:
            turbo=True
            desired_speed=self.speedControl(scan,idx,turbo) #self.esc_min+10


        desired_speed_log = desired_speed
        #print(idx,scan[639],scan[780])
        L =len(scan)
        #idx-=90

        if (idx>=0):
            idx2yaw=idx/L-0.5
            idx2yaw*=1.2
            if (idx2yaw>0.5):
                idx2yaw=0.5
            elif (idx2yaw<-0.5):
                idx2yaw=-0.5
            self.yaw=int(idx2yaw*self.yaw_range+self.neutral_yaw)

        #print(self.eStop, self.eGo,self.eStart)
        if (self.eGo or self.eStart):
            self.speedPID(desired_speed)
            if (self.eStart):
                self.throttle=self.starting_esc
                self.eStart=False
                self.eGo=True
            if (not self.eStop):
                #pass
                self.pub_esc.publish(self.throttle)
                self.pub_servo.publish(self.yaw)

        if (self.curr_speed>self.speed_record):
            self.speed_record=self.curr_speed
        tf=time.time()

        # print('turbo:',turbo,'eStp:',self.eStop,'yaw:',self.yaw,'thr:',self.throttle,'spd:',str(self.curr_speed)[0:4],'rec:', str(self.speed_record)[0:3] ,'dspd:', str(desired_speed)[0:3],'tdiff:',str(tf-ts)[0:5])
        data_to_log = {
            'yaw': self.yaw,
            'throttle': self.throttle,
            'x': desired_speed_log,
            'delta_between_callbacks': 0,
            'delta_within_callback': 0,
        }
        rospy.loginfo(
            'yaw: {yaw}'
            ' throttle: {throttle}'
            ' x: {x:.4f}'
            ' delta_between_callbacks: {delta_between_callbacks:.4f}'
            ' delta_within_callback: {delta_within_callback:.4f}'
            .format(**data_to_log)
        )



def main(args):
    rospy.init_node('tryrover_node', anonymous=True)

    if (len(args)==4):
        min_speed=args[1]
        max_speed=args[2]
        starting_esc=args[3]
    else:
        min_speed=0.5
        max_speed=6.0
        starting_esc=1580

    # print(min_speed,max_speed,starting_esc)
    ma = move_auto(min_speed,max_speed,starting_esc)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print ("Shutting down")


if __name__ == '__main__':
    main(sys.argv)
