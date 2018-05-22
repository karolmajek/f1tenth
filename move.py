import rospy, time, sys, select, termios, tty
from std_msgs.msg import String
from tf.transformations import quaternion_from_euler
from sensor_msgs.msg import Joy
from std_msgs.msg import Header, Float64
from geometry_msgs.msg import PoseStamped, TwistStamped, Vector3, Quaternion, Point
from mavros_msgs.msg import OverrideRCIn
from mavros_msgs.srv import CommandBool
from mavros_msgs.srv import CommandTOL
from mavros_msgs.srv import SetMode
from time import sleep
from picamera import PiCamera



def getKey():
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def autopilot():
    # some variables to define the behavior of the robot (hardcoded, modify at your convenience)
    throttle_channel=2
    steer_channel=0


    pub = rospy.Publisher('mavros/rc/override', OverrideRCIn, queue_size=30)
    r = rospy.Rate(10) #10hz
    msg = OverrideRCIn()
    start = time.time()
    flag=True #time flag
    throttle_ch=1500
    yaw=1300
    i=0
    camera = PiCamera()
    #camera.resolution = (160, 120)
    sleep(2)
    
    while(1):

        key = getKey()
        
        if key == 'q': #UP
            if throttle_ch > 1570:   
                throttle_ch= 1570   
            else:
                throttle_ch+=10
	elif key == 'a': #DOWN
	    if throttle_ch < 1500:       
	        throttle_ch= 1500	
            else:
                throttle_ch-=10
	elif key == 'n': #LEFT
            if yaw<600: 
                yaw=600
            else:
                yaw-=100  
		camera.capture_sequence(['/home/erle/imgs/image' + str(i).zfill(8) + '_' + str(yaw) + '_' + str(throttle_ch) + '.jpg'], use_video_port=True) 
		i+=1
	elif key == 'm': #RIGHT
            if yaw>1900: #1700
                yaw=1900
            else:
                yaw+=100  
		camera.capture_sequence(['/home/erle/imgs/image' + str(i).zfill(8) + '_' + str(yaw) + '_' + str(throttle_ch) + '.jpg'], use_video_port=True)
		i+=1
	elif key == 's': #STOP
            yaw=1300
            throttle_ch=1500
	if (key == '\x03'):
            break

        rospy.loginfo("Key: %s, Yaw: %d, Throttle: %d", key, yaw, throttle_ch)
        msg.channels[throttle_channel]=throttle_ch
        msg.channels[steer_channel]=yaw
             
        pub.publish(msg)
        
	
	r.sleep()
if __name__ == '__main__':
    rospy.init_node('tryrover_node', anonymous=True)
    rospy.wait_for_service('/mavros/set_mode')
    change_mode = rospy.ServiceProxy('/mavros/set_mode', SetMode)
    answer = change_mode(custom_mode='manual')
    settings = termios.tcgetattr(sys.stdin)
    print (answer)
    if 'True' in str(answer):
        try:
            autopilot()

        except rospy.ROSInterruptException: pass
