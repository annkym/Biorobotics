#!/usr/bin/env python
import math
import rospy
from std_msgs.msg import Empty
from std_msgs.msg import Float32
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
### A.N. Para mensajes al gripper y el brazo
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


def callbackJoy(msg):
    global speedX
    global speedY
    global yaw
    global panPos
    global tiltPos
    global spine   


    ### Read of b_button for stop the mobile base
    global stop
    ### Red button for stop of mobile base
    stop = msg.buttons[1]

    ### A.N. Boton a para abrir y cerrar el gripper
    global gripper
    gripper = msg.buttons[0]

    ### head control left Stick 
    leftStickX = msg.axes[0]
    leftStickY = msg.axes[1]

    ### base control right stick
    rightStickX = msg.axes[3]
    rightStickY = msg.axes[4]

    ### lateral moves triggers
    if msg.axes[2] == 0 and msg.axes[5] != 0:
        leftTigger = msg.axes[2]
        rightTigger = -(msg.axes[5] - 1)            ### A.N. Por que se calcula asi?

    elif msg.axes[5] == 0 and msg.axes[2] != 0:
        leftTigger = -(msg.axes[2] - 1)             ### A.N. Por que se calcula asi?
        rightTigger = msg.axes[5] 

    elif msg.axes[5] == 0 and msg.axes[2] == 0:
        leftTigger = 0
        rightTigger = 0 
    else:
        leftTigger = -(msg.axes[2] - 1)             ### A.N. Por que se calcula asi?
        rightTigger = -(msg.axes[5] - 1)            ### A.N. Por que se calcula asi?

        

    ##########################################
    #### LATERAL DISPLACEMENT left stick  ####
    magnitudTiggerDiference = math.sqrt((leftTigger*leftTigger) + (rightTigger*rightTigger))
    #print "diference: " + str(magnitudTiggerDiference)
    if magnitudTiggerDiference > 0.15:
        speedY = (leftTigger - rightTigger)/2
    else:
        speedY = 0

    ##################################
    #### HEAD CONTROL left stick  ####
    magnitudLeft = math.sqrt(leftStickX*leftStickX + leftStickY*leftStickY)
    if magnitudLeft > 0.1:
        panPos = leftStickX
        tiltPos = leftStickY
    else:
        panPos = 0
        tiltPos = 0

    panPos *= 1.5707

    if tiltPos > 0.47:
        tiltPos = 0.47
    if tiltPos < -0.7:
        tiltPos = -0.7


        
    
    ###################################################
    ### Control of mobile-base with right Stick  ######
    magnitudRight = math.sqrt(rightStickX*rightStickX + rightStickY*rightStickY)
    if magnitudRight > 0.15:
        speedX = rightStickY
        yaw = rightStickX
    else:
        speedX = 0
        yaw = 0
	


def main():
    global leftSpeed
    global rightSpeed
    global panPos 
    global tiltPos

    global speedX
    global speedY
    global yaw
    global stop
    global mov_spine
    global spine	
    ###A.N.
    global gripper

    leftSpeed = 0
    rightSpeed = 0
    panPos = 0
    tiltPos = 0
    speedY = 0
    speedX = 0
    yaw = 0
    spine =0
    mov_spine=False
    ### A.N.
    stop = 0
    gripper = 0
    open = False
    
    msgSpeeds = Float32MultiArray()
    msgHeadPos = Float32MultiArray()
    msgSpine = Float32()
    msgTwist = Twist()
    ### A.N.
    msgJoint = JointTrajectory()
    
    print "INITIALIZING JOYSTICK TELEOP BY EDD-II...  "
    rospy.init_node("joystick_teleop")
    
    rospy.Subscriber("/joy", Joy, callbackJoy)

    ## TODO: VERIFY--- topics names to head and spine
    pubSpin = rospy.Publisher("/hardware/torso/goal_spine", Float32, queue_size=1)
    
    pubTwist = rospy.Publisher("/hsrb/opt_command_velocity", Twist, queue_size =1)
    pubHeadPos = rospy.Publisher("/hsr_pumas/head/goal_pose", Float32MultiArray, queue_size=1)

    ###A.N.
    pubGrip = rospy.Publisher("/hsrb/gripper_trajectory_controller/command", JointTrajectory, queue_size=1)

    
    loop = rospy.Rate(10)
    while not rospy.is_shutdown():
        if math.fabs(speedX) > 0 or math.fabs(speedY) > 0 or math.fabs(yaw) > 0:
            msgTwist.linear.x = speedX
            msgTwist.linear.y = speedY/2.0
            msgTwist.linear.z = 0
            msgTwist.angular.z = yaw
            pubTwist.publish(msgTwist)

        ### A.N. Para detener con el boton B (stop)
        if stop > 0:
            print "stop % d" % stop
            msgTwist.linear.x = 0
            msgTwist.linear.y = 0
            msgTwist.linear.z = 0
            msgTwist.angular.z = 0
            pubTwist.publish(msgTwist)
        if gripper > 0:
            print "gripeando %f" % gripper
            msgJoint.joint_names=["hand_l_proximal_joint", "hand_r_proximal_joint"]
            if open :
                positions=[0.0,0.0]
                open = False
                print "Cerrando"
            else:
                positions = [0.611,-0.611]
                open = True
                print "Abriendo"
            punto =JointTrajectoryPoint()
            punto.positions = positions
            puntos=[punto]
            msgJoint.points = puntos
            pubGrip.publish(msgJoint)
            print "Terminando 1"



        if math.fabs(panPos) > 0 or math.fabs(tiltPos) > 0:
            msgHeadPos.data = [panPos, tiltPos]
            pubHeadPos.publish(msgHeadPos)

	if spine <= 0.51 and spine >= -0.51 and mov_spine==True:
	    if(spine_button == 1 and spine < 0.5 ):
                spine=spine+0.01
    	    if(spine_button ==-1 and spine > -0.5):
                spine=spine-0.01
	    msgSpine.data = spine
	    pubSpin.publish(msgSpine)
	   

        loop.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
