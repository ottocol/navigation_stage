# -*- coding: utf-8 -*-
# from __future__ import print_function

import rospy
import smach_ros
import math
from smach import State,StateMachine
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Int32
from geometry_msgs.msg import Twist

TOPIC_VEL = "/cmd_vel"
TOPIC_SCAN = '/base_scan'
TOPIC_COLOR = '/color_detected'


#Nos fijamos en el ángulo a 30 y -30 grados, podéis cambiarlo si quereis
ANG_IZQ = 30*math.pi/180.0
ANG_DER = -ANG_IZQ



class WanderAndDetect(State):
    def __init__(self):
        State.__init__(self, outcomes=['color_detected'])
        self.color_detected = False
        self.pub = rospy.Publisher(TOPIC_VEL, Twist, queue_size=5)
    
    def execute(self, userdata):
        self.subScan = rospy.Subscriber(TOPIC_SCAN, LaserScan, self.laser_callback)
        self.subColor = rospy.Subscriber(TOPIC_COLOR, Int32 , self.color_detected_calback)
        rate = rospy.Rate(10)

        
        while not self.color_detected:
            rate.sleep()

        #normalmente en un nodo de ROS convencional no nos desuscribimos
        #porque se hace automáticamente al acabar el nodo, pero esto es un estado
        #de la máquina de estados y mejor "limpiar" todo antes de saltar a otro estado
        if self.color_detected:
            self.subScan.unregister()
            self.subColor.unregister()

        
        return "color_detected"
        
    
    def laser_callback(self, msg):
        #cuáles son los rayos de laser que nos interesan?
        pos_izq = int((ANG_IZQ-msg.angle_min)/msg.angle_increment)
        pos_der = int((ANG_DER-msg.angle_min)/msg.angle_increment) 
        #FALTA: calcular la velocidad angular en z y lineal en x adecuadas
        #a las distancias detectadas
        print("Izq", msg.ranges[pos_izq], " Der: ", msg.ranges[pos_der])

        cmd = Twist()
        #FALTA: "rellenar" cmd con valores de velocidad angular en z y lineal en x
        self.pub.publish(cmd)

    def color_detected_calback(self, msg):
        self.color_detected = True


if __name__ == '__main__':
    rospy.init_node("practica3")
    sm = StateMachine(outcomes=['end'])
    with sm:
        #en la versión final la transición deberá ser al estado "volver a la base"
        StateMachine.add('WanderAndDetect', WanderAndDetect(), 
           transitions={
               'color_detected':'end'})
    
    sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
    sis.start()
    sm.execute()
    rospy.spin()   
