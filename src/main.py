#! /usr/bin/python3

import rospy
import tf 
import rospy
import actionlib

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from projeto_11.srv import Camservice, CamserviceRequest 
from time import sleep
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectory,JointTrajectoryPoint
from math import pi


class myRobot(): 

    def __init__(self):

        self.euler = 0
        self.sub_odometria = rospy.Subscriber('/mobile_base_controller/odom', Odometry, self.callback_odometria)
        self.sub_laser = rospy.Subscriber('/scan_raw', LaserScan, self.callback_laser)
        self.pub_cabeca = rospy.Publisher('/head_controller/command', JointTrajectory, queue_size=1)
        self.pub_velocidade =rospy.Publisher('/nav_vel', Twist, queue_size=1)
        self.camera_res = rospy.ServiceProxy('detect', Camservice)

        self.velocidade = Twist()
        self.rotacao = rospy.Rate(15)
        self.centro = 10 
        self.direita = 0
        self.esquerda = 0
        self.estado = 'andar'
            
    def callback_odometria(self, msg):
        quaternion = (msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)
        self.euler = tf.transformations.euler_from_quaternion(quaternion)[2]

    def callback_laser(self, msg):
        self.centro = msg.ranges[333]
        self.direita = msg.ranges[90]
        self.esquerda = msg.ranges[570]

    def mover(self):
        self.velocidade.linear.x = 0.5
        self.pub_velocidade.publish(self.velocidade)

    def parar(self):
        self.velocidade.linear.x = 0
        self.pub_velocidade.publish(self.velocidade)

    def virar(self, sensor): 
        if sensor == 'direita':
            guidance = self.euler - 1.57 
        elif sensor == 'esquerda':
            guidance = self.euler + 1.57
        else:
            guidance = self.euler + 0.0
        self.estado = 'girar' 
        
        while abs(self.euler - guidance) > 0.1:
            self.velocidade.angular.z = -0.5 if sensor == 'direita' else 0.5
            self.pub_velocidade.publish(self.velocidade)
            self.rotacao.sleep()
        self.velocidade.angular.z = 0.0
        self.pub_velocidade.publish(self.velocidade)
        self.estado = 'parar de girar'

    def movimento_cabeca(self, direcao):
        if direcao == 'esquerda':
            x = 2
            y = 0
        if direcao == 'direita':
            x = -2
            y = 0   
        if direcao == 'centro':
            x = 0
            y = 0
        msg = JointTrajectory()
        msg.joint_names = ['head_1_joint', 'head_2_joint']  
        point = JointTrajectoryPoint()
        point.positions = [x, y]
        point.velocities = [0.0, 0.0]
        point.time_from_start = rospy.Duration(1.0)
        msg.points = [point]
        self.pub_cabeca.publish(msg)
        self.rotacao.sleep()   
    
    def servico_camera(self):
        rospy.wait_for_service('detect')
        try:
            serv_cam = rospy.ServiceProxy('detect', Camservice)
            request = CamserviceRequest()
            response = serv_cam(request)
            return response.camera 
        except rospy.ServiceException as e:
            print ("Service call failed: %s"%e )  

    def dar_xau(self):
        arm_client = actionlib.SimpleActionClient("/arm_controller/follow_joint_trajectory", FollowJointTrajectoryAction)
        arm_client.wait_for_server()

        arm_trajectory = JointTrajectory()
        arm_trajectory.joint_names = ["arm_1_joint", "arm_2_joint", "arm_3_joint", 
                                    "arm_4_joint", "arm_5_joint", "arm_6_joint", "arm_7_joint"]
        points = [[0.0, pi/2, 0.0, pi/4, 0.0, 0.0, 0.0],
                [0.0, pi/2, 0.0, -pi, 0.0, 0.0, 0.0],
                [0.0, pi/2, 0.0, pi/4, 0.0, 0.0, 0.0],
                [0.0, pi/2, 0.0, -pi, 0.0, 0.0, 0.0],
                [0.0, pi/2, 0.0, pi/4, 0.0, 0.0, 0.0],
                [0.0, pi/2, 0.0, -pi, 0.0, 0.0, 0.0],
                [0.0, pi/2, 0.0, pi/4, 0.0, 0.0, 0.0],
                [0.0, pi/2, 0.0, -pi, 0.0, 0.0, 0.0]]
        time_from_start = rospy.Duration(0.0)
        for point in points:
            joint_point = JointTrajectoryPoint()
            joint_point.positions = point
            joint_point.time_from_start = time_from_start
            arm_trajectory.points.append(joint_point)
            time_from_start += rospy.Duration(1.5) 
        arm_goal = FollowJointTrajectoryGoal()
        arm_goal.trajectory = arm_trajectory
        
        arm_client.send_goal(arm_goal)
        arm_client.wait_for_result()                 

    def decisao(self):
       
        if self.centro > 0.8 and self.estado == 'andar':
            self.mover()
        elif self.centro <= 0.8 and self.estado == 'andar':
            self.parar()
            self.estado = 'parar'
        if self.estado == 'parar':
            if self.esquerda > 1.5:
                self.estado = 'girar a esquerda'
            if self.direita > 1.5:
                self.estado = 'girar a direita'
            if self.esquerda > 1.5 and self.direita > 1.5:
                self.estado = 'ligar a camera'  

        if self.estado == 'girar a esquerda':
            self.virar('esquerda')
        if self.estado == 'girar a direita':
            self.virar('direita') 

        if self.estado == 'parar de girar':
            self.estado = 'andar'            

        if self.estado == 'ligar a camera':
            self.movimento_cabeca('esquerda')
            sleep(3)
            esquerda = self.servico_camera()  
            self.movimento_cabeca('direita')
            sleep(3)
            direita = self.servico_camera()
            self.movimento_cabeca('centro')
            sleep(3)
            if esquerda == True and direita == False:
                self.virar('esquerda')
            elif direita == True and esquerda == False:
                self.virar('direita')
            else:
                self.dar_xau()
                self.estado = 'acabou'
                print('Fim do programa')

if __name__ == '__main__':
    rospy.init_node('projeto_11') 
    tiago = myRobot()
    while not rospy.is_shutdown():
        tiago.decisao()
        tiago.rotacao.sleep()                   




            
