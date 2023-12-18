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

from tf.transformations import quaternion_from_euler
    
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

# Objetos de Service, Publisher e Subscriber
arm = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
set_mode_srv = rospy.ServiceProxy('/mavros/set_mode', SetMode)
local_position_pub = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size=10)
state_sub = rospy.Subscriber("/mavros/state", State, state_callback)
cmd_vel_pub = rospy.Publisher('/mavros/setpoint_velocity/cmd_vel', TwistStamped, queue_size=10)
pose_sub = rospy.Subscriber("/mavros/local_position/pose", PoseStamped, pose_callback)
extended_state_sub = rospy.Subscriber("/mavros/extended_state", ExtendedState, extended_state_callback)


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

# drone deve subir (quanto?) e girar 360 graus (como garantir que ele gira 360 graus?)

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
"""

rospy.loginfo("Subindo")
goal_pose.pose.position.z = 5
while not rospy.is_shutdown() and abs(goal_pose.pose.position.z - current_pose.pose.position.z) > TOL:
    local_position_pub.publish(goal_pose)
    rate.sleep()

# girando 360 graus

rospy.loginfo("Girando 360 graus")







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