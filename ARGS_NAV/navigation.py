#!/usr/bin/env python
import rospy
import sys
import time

from geometry_msgs.msg import *
from std_msgs.msg import *
from tf2_msgs.msg import *


pub_objectif = rospy.Publisher('move_base_simple/goal',PoseStamped,queue_size=1)
pub_pose = rospy.Publisher('initialpose',PoseWithCovarianceStamped,queue_size=1)
pub_debug = rospy.Publisher('DEBUG',String,queue_size=1)
#-------------------------------------------------------------------------------
# Nom fonction : VerifDep()
# But fonction : Permet de verifier si on est bien arrive sur l'objectif
#                et permet de publish un ack
#-------------------------------------------------------------------------------


class navigation:
    global pub_debug
    global pub_pose
    global pub_objectif
    #-------------------------------------------------------------------------------
    # Nom fonction : __init__()
    # But fonction : Init de la classe navigation
    #-------------------------------------------------------------------------------
    def __init__(self):
        #Initialisation ros Publisher et Ros Subscriber
        self.Grille_Jeu_X = Float64MultiArray()
        self.Grille_Jeu_Y = Float64MultiArray()
        self.JeuMorpion = Int8MultiArray()
        self.Goal = Int8MultiArray()
        self.FlagY = False
        self.FlagX = False
        self.FlagG = False

         #point ou on veut jouer - Venant du node IA
        self.Grille_Jeu_X_Sub = rospy.Subscriber("Grille_Jeu_Y",Float64MultiArray,self.Callback_X,queue_size=1)
        self.Grille_Jeu_Y_Sub = rospy.Subscriber("Grille_Jeu_X",Float64MultiArray,self.Callback_Y,queue_size=1)
        self.Case_Jeu_Sub = rospy.Subscriber("Case_Jeu",Int8MultiArray,self.Callback_Goal,queue_size = 1)

    def Pose_Initial(self):
        global pub_debug
        pub_debug.publish('Pose_Initial')
        Initial_Pose = PoseWithCovarianceStamped()
        Initial_Pose.header.stamp = rospy.Time.now()
        Initial_Pose.header.frame_id = 'map'

        Initial_Pose.pose.pose.position.x = 0.0
        Initial_Pose.pose.pose.position.y = 0.0
        Initial_Pose.pose.pose.position.z = 0.0

        Initial_Pose.pose.pose.orientation.x = 0.0
        Initial_Pose.pose.pose.orientation.y = 0.0
        Initial_Pose.pose.pose.orientation.z = 0.0
        Initial_Pose.pose.pose.orientation.w = 1.0

        pub_pose.publish(Initial_Pose)

    #-------------------------------------------------------------------------------
    # Nom fonction : Callback_X()
    # But fonction : Permet de recuperer la valeur des coo en X de la grille de jeu
    #-------------------------------------------------------------------------------

    def Callback_X(self,data):
        self.Grille_Jeu_X.data = data.data
        self.FlagX = True
        pub_debug.publish('Callback_X')

    #-------------------------------------------------------------------------------
    # Nom fonction : Callback_Y()
    # But fonction : Permet de recuperer la valeur des coo en Y de la grille de jeu
    #-------------------------------------------------------------------------------

    def Callback_Y(self,data):
        self.Grille_Jeu_Y.data = data.data
        self.FlagY = True
        pub_debug.publish('Callback_Y')

    #-------------------------------------------------------------------------------
    # Nom fonction : Callback_Goal()
    # But fonction : Permet de recuperer les cases a jouer
    #-------------------------------------------------------------------------------

    def Callback_Goal(self,data):
        self.Goal = data.data
        self.FlagG = True
        pub_debug.publish('Callback_Goal')


    #-------------------------------------------------------------------------------
    # Nom fonction : Nav2goal()
    # But fonction : Envoyer une commande au robot pour qu'il se deplace vers le
    #                   point objectif
    #------------------------------------------------------------------------------


    def Nav2goal(self):
        global pub_debug
        pub_debug.publish('Nav2goal')
        #data                       #Grille de jeu (case objectif)
        # 6 | 7 | 8                 # C0 | C1 | C2
        #-----------                #------------
        # 3 | 4 | 5     ------->    # B0 | B1 | B2
        #-----------                #-------------
        # 0 | 1 | 2                 # A0 | A1 | A2

        if self.FlagY == True and self.FlagX == True and self.FlagG == True:
            print("je suis la Nav2goal apres if")
            pub_debug.publish('Yalah')

            for Case_Jouee in self.Goal.data:
                # Permet d avoir les coos de la case ou on souhaite jouer
                goal = PoseStamped()
                goal.header.stamp = rospy.Time.now()
                goal.header.frame_id = 'map'

                goal.pose.position.x = self.Grille_Jeu_X.data[Case_Jouee]
                goal.pose.position.y = self.Grille_Jeu_Y.data[Case_Jouee]

                goal.pose.position.z = 0.0
                goal.pose.orientation.x = 0.0
                goal.pose.orientation.y = 0.0
                goal.pose.orientation.z = 0.0
                goal.pose.orientation.w = 1.0

                pub_objectif.publish(goal)
                time.sleep(20)


def main(arg):

    ic=navigation()

    rospy.init_node('Navigation', anonymous=True)
    ic.Pose_Initial()
    ic.Nav2goal()

    rospy.spin()


if __name__ == '__main__':
    main(sys.argv)

    ''' PROBLEME DANS LES NODES, lors de la recuperation des info des autres nodes'''
