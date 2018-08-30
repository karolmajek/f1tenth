#!/usr/bin/env python3.4

import numpy as np
import time
import math
import sys
import rospy, time, sys, select, termios, tty
from std_msgs.msg import Header, Float64
from mavros_msgs.msg import OverrideRCIn
from mavros_msgs.srv import CommandBool
from mavros_msgs.srv import CommandTOL
from mavros_msgs.srv import SetMode
from sensor_msgs.msg import LaserScan


class move_auto:
    def __init__(self):

        self.pub = rospy.Publisher('mavros/rc/override', OverrideRCIn,
                                   queue_size=5)
        # self.sub = rospy.Subscriber("/camera/image/compressed", CompressedImage, self.callback, queue_size = 1)
        self.sub = rospy.Subscriber("/scan", LaserScan, self.laserCallback,
                                    queue_size=1)

        
        self.throttle = 0
        self.yaw = 1570
        self.safe_obstacle_distance1=0.3
        self.safe_obstacle_distance2=0.3
        self.non_cont_dist=0.1 #key perf vs acc parameter

        #self.kp=14.0*5
        #self.kd=0.09*10
        #self.prev_error=0.0
        self.angle_increment=math.radians(0.25) #0.004363323096185923

        self.num_pnts_to_chk=20
        self.num_pnts_to_store=1080
        self.sin_alfa=np.zeros(self.num_pnts_to_store)


        for i in range(self.num_pnts_to_store):
            self.sin_alfa[i]=math.sin(math.radians((i+1)*0.25))
        

        self.throttle_channel = 2
        self.steer_channel = 0
        #self.thetaR=50
        #self.thetaL=180-self.thetaR
        #self.desired_trajectory=0.0

        self.msg = OverrideRCIn()
    def checkIfReachable(self,r1,r2,alfa):
        if (r1-self.safe_obstacle_distance1<r2):
            return True
        else:
            return (r2*self.sin_alfa[alfa]>self.safe_obstacle_distance2)

    
    def steerMAX(self,scan):
        
        scan= scan[180:-180]

        scan2=np.copy(scan)
        
        idx=0

        isReachable = False

        segs=[]
        segs.append(0)
        segs.append(len(scan)-1)

        
    
        for i in range(1,len(scan)):
            if (abs(scan[i]-scan[i-1])> self.non_cont_dist):
                segs.append(i)
                segs.append(i-1)        

        
        while (not isReachable):
            idx=np.argmax(scan2)
            
            for s in segs:
                if (s != idx):
                    #print(abs(s-idx),s,idx)
                    if not (self.checkIfReachable(scan[idx],scan[s],abs(s-idx))):
                        scan2[idx]=-1
                        break
            if (scan2[idx] != -1):
                isReachable=True
        
        yaw=idx/len(scan)-0.5
        #if (yaw < 0.1 and yaw >-0.1):
        #    yaw*=5
        if (yaw>0.5):
            yaw=0.5
        if (yaw<-0.5):
            yaw=-0.5

       

        return int((yaw)*1200)+1570



    def checkIfContinous(self,scan,angle):

        segment_length=20
        r_prev=scan[0]
        index=0
        apex_cands=[]
        apexes=[]
        apex_cnt=0


        for r in scan:


            if (abs(r-r_prev) >1.0 and index > segment_length and index < len(scan)-segment_length and min(r,r_prev)<3.0):
                #index > segment_length and index < len(scan)-segment_length ):
                apex_cands.append(index)
            index+=1
            r_prev=r
    

        for i in apex_cands:
            flag=1
            for j in range(1,self.num_pnts_to_chk):
                if (abs(scan[i-1-j]-scan[i-1-j-1])> 0.4 or abs(scan[i+1+j]-scan[i+1+j-1])> 0.4):
                    flag=0
            if (flag==1):
                apex_cnt+=1

        if (apex_cnt > 0 or scan[540]< 1.0):
            return False
        else:
            return True   



#    def getRangeR(self, data, theta):
# 
#        car_theta = math.radians(theta) - math.pi / 2
#
#        
#        
#        #if car_theta > 3 * math.pi / 4:
#        #    car_theta = 3 * math.pi / 4
#        #elif car_theta < -3 * math.pi / 4:
#        #    car_theta = -3 * math.pi / 4
#    
#        float_index = (car_theta + 3 * math.pi / 4) / self.angle_increment
#        index = int(float_index)
#        return data[index]

#    def getRangeL(self, data, theta):
# 
#        car_theta = math.radians(theta) - math.pi / 2 
#        
#        #if car_theta > 3 * math.pi / 4:
#        #    car_theta = 3 * math.pi / 4
#        #elif car_theta < -3 * math.pi / 4:
#        #    car_theta = -3 * math.pi / 4
#    
#        float_index = (car_theta + 3 * math.pi / 4) / self.angle_increment
#        index = int(float_index)
#        #print("index",index)
#        return data[index]
    
    
#    def steerPID(self, data):
#        aR = self.getRangeR(data, self.thetaR)
#        bR = self.getRangeR(data, 0)
#        swingR = math.radians(self.thetaR)
#        alphaR = math.atan2( aR * math.cos(swingR) - bR , aR * math.sin(swingR) )
#        ABR = bR * math.cos(alphaR)
#        ACR = 1
#        CDR = ABR + ACR * math.sin(alphaR)

#        aL = self.getRangeL(data, 180)
#        bL = self.getRangeL(data, self.thetaL)
#        swingL = math.radians(self.thetaL)
#        alphaL = math.atan2( aL * math.cos(swingL) - bL , aL * math.sin(swingL) )
#        ABL = bL * math.cos(alphaL)
#        ACL = 1
#        CDL = ABL -ACL * math.sin(alphaL)

        


#        pid_error = CDR-CDL- self.desired_trajectory

        
#        error = pid_error * self.kp
#        errordot = self.kd * (pid_error - self.prev_error)

#        angle = error + errordot

#        if angle > 100:
#            angle = 100
#        elif angle < -100:
#            angle = -100

#        self.prev_error = pid_error        
    
#        return int(1570-angle*6)

        
        #print ( CDL, CDR, abs(CDL-CDR))
        #print ('yaw: %.2f pid_error: %.2f error: %.2f errordot:%.2f angle %.2f' % (self.yaw, pid_error, error, errordot, angle))
    def laserCallback(self, data):


        #print(data.angle_increment)
        ts=time.time()

        scan= np.array(data.ranges)
        scan[scan>10]=10 
        scan[scan<0]=0 
    
        isCont=self.checkIfContinous(scan,data.angle_min)
    
        if (True): #(isCont):
            #self.yaw=self.steerPID(scan)
            #steering='PID'
        #else:
            self.yaw=self.steerMAX(scan)
            steering='MAX'
        #print(yaw)
        tf=time.time()
        print(steering,self.yaw,tf-ts)    
        
	
        self.msg.channels[self.throttle_channel] = self.throttle
        self.msg.channels[self.steer_channel] = self.yaw       
        self.pub.publish(self.msg)

        

def main(args):
    rospy.init_node('tryrover_node', anonymous=True)
    ma = move_auto() #args[1]
    rospy.wait_for_service('/mavros/set_mode')
    change_mode = rospy.ServiceProxy('/mavros/set_mode', SetMode)
    answer = change_mode(custom_mode='manual')
    print (answer)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print ("Shutting down ROS Image feature detector module")


if __name__ == '__main__':
    main(sys.argv)
