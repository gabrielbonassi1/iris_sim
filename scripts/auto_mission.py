#!/usr/bin/python3
# -*- coding:utf-8 -*-

# ===========================
# === CONFIGURANDO O NODE ===
# ===========================

import rospy # Biblioteca do ROS para Python

# Classes das mensagens e servicos
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import TwistStamped
from mavros_msgs.srv import SetMode, CommandBool
from mavros_msgs.msg import State
from mavros_msgs.msg import ExtendedState
from std_msgs.msg import Bool
import numpy as np
import os

from tf.transformations import quaternion_from_euler, euler_from_quaternion
    
# Inicializa o node
rospy.init_node("auto_mission")

# ================================
# === PUBLISHERS E SUBSCRIBERS ===
# ================================

# Objetos de comandos e estados
current_state = State()
current_pose = PoseStamped()
goal_pose = PoseStamped()
extended_state = ExtendedState()
cmd_vel = TwistStamped()
# Frequencia de publicacao do setpoint
rate = rospy.Rate(20)
dois_pi = 2*np.pi

def multiple_rate_sleep(n):
    for i in range(n):
        rate.sleep()

# Funções de callback
def state_callback(msg):
    global current_state
    current_state = msg

def pose_callback(msg):
    global current_pose
    current_pose = msg

def extended_state_callback(msg):
    global extended_state
    extended_state = msg

def found_baloon_callback(msg):
    global found_baloon
    found_baloon = msg

def centralized_baloon_callback(msg):
    global centralized_baloon
    centralized_baloon = msg  

def drone_reference(direction_array,yaw):
    theta = yaw 
    rotation_matrix = np.array([[np.cos(theta), -np.sin(theta), 0],
                                [np.sin(theta), np.cos(theta), 0],
                                [0, 0, 1]])    
    vector = np.matmul(rotation_matrix,direction_array)
    return vector

def centralize_baloon():
    # Run the other script to centralize the baloon
    #subprocess.run(["python3", "baloon_centralize.py"])    
    cwd = os.getcwd()
    with open(cwd + "/baloon_centralize.py") as f:
        exec(f.read())

def track_baloon():
    # Run the other script to centralize the baloon
    #subprocess.run(["python3", "baloon_centralize.py"])    
    cwd = os.getcwd()
    with open(cwd + "/baloon_tracking.py") as f:
        exec(f.read())

# Objetos de Service, Publisher e Subscriber
arm = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
set_mode_srv = rospy.ServiceProxy('/mavros/set_mode', SetMode)
local_position_pub = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size=10)
state_sub = rospy.Subscriber("/mavros/state", State, state_callback)
cmd_vel_pub = rospy.Publisher('/mavros/setpoint_velocity/cmd_vel', TwistStamped, queue_size=10)
pose_sub = rospy.Subscriber("/mavros/local_position/pose", PoseStamped, pose_callback)
extended_state_sub = rospy.Subscriber("/mavros/extended_state", ExtendedState, extended_state_callback)
found_baloon_sub = rospy.Subscriber("/baloon_tracking/found_baloon", Bool, found_baloon_callback)
centralized_baloon_sub = rospy.Subscriber("/baloon_tracking/centralized_baloon", Bool, centralized_baloon_callback)
# =============================
# === PREPARACAO PARA O VOO ===
# =============================

# Espera a conexao ser iniciada
rospy.loginfo("Esperando conexao com FCU")
while not rospy.is_shutdown() and not current_state.connected:
    rate.sleep()

# Publica algumas mensagens antes de trocar o modo de voo
for i in range(100):
    local_position_pub.publish(goal_pose)
    rate.sleep()

# Coloca no modo Offboard
last_request = rospy.Time.now()
if (current_state.mode != "OFFBOARD"):
    result = set_mode_srv(0, "OFFBOARD")
    rospy.loginfo("Alterando para modo Offboard")
    while not rospy.is_shutdown() and current_state.mode != "OFFBOARD" and (rospy.Time.now() - last_request > rospy.Duration(1.0)):
        result = set_mode_srv(0, "OFFBOARD")
    rospy.loginfo("Drone em modo Offboard")
else:
    rospy.loginfo("Drone já está em modo Offboard")

# Arma o drone
if (not current_state.armed):
    result = arm(True)
    rospy.loginfo("Armando o drone")
    while not rospy.is_shutdown() and not current_state.armed:
        result = arm(True)
    rospy.loginfo("Drone armado")
else:
    rospy.loginfo("Drone ja armado")

# =============================
# === MOVIMENTACAO DO DRONE ===
# =============================

# Tolerancia de posicionamento
TOL = 0.1
def subir():
    # subindo:
    """
    Formato da mensagem geometry_msgs/PoseStamped, que controla a posição do drone
    std_msgs/Header header
      uint32 seq
      time stamp
      string frame_id
    geometry_msgs/Pose pose
      geometry_msgs/Point position
        float64 x
        float64 y
        float64 z
      geometry_msgs/Quaternion orientation
        float64 x
        float64 y
        float64 z
        float64 w

    Pra subir, queremos mexer no geometry_msgs.pose.position.z
    Para virar em z, mexemos no geometry_msgs.pose.orientation.z
    """
    rospy.loginfo("Subindo")
    goal_pose.pose.position.z = current_pose.pose.position.z + 5
    while not rospy.is_shutdown() and abs(goal_pose.pose.position.z - current_pose.pose.position.z) > TOL:
        local_position_pub.publish(goal_pose)
        rate.sleep()


def girar():
    rospy.loginfo("Girando 360 graus")
    # Defina as coordenadas de posição (mantenha a altitude constante)
    yaw = 0.0
    while not rospy.is_shutdown() and yaw <= 6.28 and found_baloon.data == False:
        quaternion = quaternion_from_euler(0, 0, yaw)
        # define as coordenadas da orientação em quaternion
        goal_pose.pose.orientation.x = quaternion[0]
        goal_pose.pose.orientation.y = quaternion[1]
        goal_pose.pose.orientation.z = quaternion[2]
        goal_pose.pose.orientation.w = quaternion[3]
        local_position_pub.publish(goal_pose)
        rate.sleep()
        yaw += 0.1

def ir_balao():
    centralize_baloon()
    vel = 1.0
    yaw = euler_from_quaternion([current_pose.pose.orientation.x, current_pose.pose.orientation.y, current_pose.pose.orientation.z, current_pose.pose.orientation.w])[2]
    direction_array = np.array([1,0,0])
    vel_vector = drone_reference(vel*direction_array,yaw)
    cmd_vel.twist.linear.x = vel_vector[0]
    cmd_vel.twist.linear.y = vel_vector[1]
    cmd_vel.twist.linear.z = vel_vector[2]
    cmd_vel.twist.angular.z= 0.0
    while not rospy.is_shutdown(): # Necessidade: um sensor de proximidade pra dizer se chegou no balão
        cmd_vel_pub.publish(cmd_vel)
        rate.sleep()

def fim():
    # Coloca no modo Land
    if (current_state.mode != "AUTO.LAND"):
        result = set_mode_srv(0, "AUTO.LAND")
        rospy.loginfo("Alterando para modo Land")
        while not rospy.is_shutdown() and current_state.mode != "AUTO.LAND":
            result = set_mode_srv(0, "AUTO.LAND")
        rospy.loginfo("Drone em modo Land")
    else:
        rospy.loginfo("Drone ja esta em modo Land")

    #Espera Pousar
    land_state_on_ground = 1
    while not rospy.is_shutdown() and extended_state.landed_state != land_state_on_ground:
        if(extended_state.landed_state == 1):
            rospy.loginfo("Pousado no solo. Altura = " + str(current_pose.pose.position.z) + " m")
        elif(extended_state.landed_state == 2):
            rospy.loginfo("Voando. Altura = " + str(current_pose.pose.position.z) + " m")
        elif(extended_state.landed_state == 3):
            rospy.loginfo("Decolando. Altura = " + str(current_pose.pose.position.z) + " m")
        elif(extended_state.landed_state == 4):
            rospy.loginfo("Pousando. Altura = " + str(current_pose.pose.position.z) + " m") 
        multiple_rate_sleep(10)


    #Printa 3 vezes que pousou no solo
    i=0
    while not rospy.is_shutdown() and i<3:
        if(extended_state.landed_state == 1):
            rospy.loginfo("Pousado no solo. Altura = " + str(current_pose.pose.position.z) + " m")
        elif(extended_state.landed_state == 2):
            rospy.loginfo("Voando. Altura = " + str(current_pose.pose.position.z) + " m")
        elif(extended_state.landed_state == 3):
            rospy.loginfo("Decolando. Altura = " + str(current_pose.pose.position.z) + " m")
        elif(extended_state.landed_state == 4):
            rospy.loginfo("Pousando. Altura = " + str(current_pose.pose.position.z) + " m") 
        i+=1
        multiple_rate_sleep(10)  

    #Espera o usuario pressionar Ctrl+C
    while not rospy.is_shutdown():
        print("Pressione Ctrl+C para encerrar a simulacao")
        multiple_rate_sleep(100)

# ==============================
# ===== MAQUINA DE ESTADOS =====
# ==============================
state = "init"
vezes_count = 0
limite = 3
count = 0
acabou = False
while acabou == False:
    if state == "init":
        state = "subir"
    elif state == "subir":
        subir()
        state = "girar"
    elif state == "girar":
        girar()
        if found_baloon.data == True:
            state = "ir_balao"
        elif vezes_count < limite-1:
            state = "subir"
            vezes_count += 1
        else:
            state = "fim"
    elif state == "ir_balao":
        ir_balao()
        count += 1
        state = "ir_balao"
    elif state == "ir_balao" and count == 100:
        state = "fim"
    elif state == "fim":
        fim()
        acabou = True
        break

