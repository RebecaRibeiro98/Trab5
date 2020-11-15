import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
import tf
import math

matriculas = [2017003772, 2016000168, 2017001160, 2016018028]
freqMat = 0.0
timeMat = 0.0
estado = "busca"
velAng=0.3


kp = 0.5
ki = 0.015
kd = 0.02

lastError = 0
sumError = 0

odom = Odometry()
scan = LaserScan()

rospy.init_node('cmd_node')

# Auxiliar functions ------------------------------------------------
def getAngle(msg):
    quaternion = msg.pose.pose.orientation
    quat = [quaternion.x, quaternion.y, quaternion.z, quaternion.w]
    euler = tf.transformations.euler_from_quaternion(quat)
    yaw = euler[2]*180.0/math.pi
    return yaw
    
def mediaSomaMatriculas(arrayMat):
    avg = 0
    for matricula in arrayMat:
        sumMat=0
        for i in str(matricula):
            sumMat += int(i)
            
        avg+=sumMat
        
    avg = float(avg)/len(arrayMat)
    return avg

freqMat = mediaSomaMatriculas(matriculas)
timeMat = 1.0/freqMat
print('Frequencia: '+str(freqMat)+' Hz')

# CALLBACKS ---------------------------------------------------------
def odomCallBack(msg):
    global odom
    odom = msg
    
def scanCallBack(msg):
    global scan
    scan = msg
#--------------------------------------------------------------------

# TIMER - Control Loop ----------------------------------------------
def timerCallBack(event):
    global lastError, sumError, estado, velAng
    
    setpoint = 0.5
    scan_len = len(scan.ranges)
    msg = Twist()
   
    if not(scan_len > 0):
        msg.angular.z = 0
        msg.linear.x = 0
        
    # POSICIONA DIRECAO ---------------------------------   
    elif estado == 'busca':
        print('Buscando...')
       
        if min(scan.ranges[scan_len-5 : scan_len+5]) < 100:
           
            estado = 'avanca'
            lastError = sumError = 0
            msg.angular.z = 0
        else:
            if min(scan.ranges[scan_len-15 : scan_len+15]) < 100:
                msg.angular.z = velAng*0.5
            else:
                msg.angular.z = velAng
            

    elif estado == 'avanca':
  
    # AVANCA --------------------------------
    
        read = min(scan.ranges[scan_len-15 : scan_len+15])
        P=I=D=0
        if read < 100:
            error = -(setpoint - read)
            varError = (error-lastError)/timeMat
            sumError+=error*timeMat
            
            lastError = error   
            
            P = kp*error
            I = ki*sumError
            D = kd*varError
        
        control = P+I+D
        print(read, P, I, D)
        
        if control > 1:
            control = 1
        elif control < -1:
            control = -1
        
          
        msg.linear.x = control
        msg.angular.z = 0


        if read>100:
            estado = 'busca'
        
    
    
    
    pub.publish(msg)
    

pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
odom_sub = rospy.Subscriber('/odom', Odometry, odomCallBack)
scan_sub = rospy.Subscriber('/scan', LaserScan, scanCallBack)

timer = rospy.Timer(rospy.Duration(timeMat), timerCallBack)

rospy.spin()