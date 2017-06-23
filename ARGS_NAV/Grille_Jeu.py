#!/usr/bin/env python
import rospy
import sys
from geometry_msgs.msg import *
from std_msgs.msg import *
from tf2_msgs.msg import *

pub_grille_jeu_X = rospy.Publisher('Grille_Jeu_X',Float64MultiArray,queue_size=1)
pub_grille_jeu_Y = rospy.Publisher('Grille_Jeu_Y',Float64MultiArray,queue_size=1)

MILLEU_CARRE = 0.05

class Case:
    def __init__(self):
        self.x = 0
        self.y = 0

    def ecrire(self,coo_x,coo_y):
        self.x = coo_x
        self.y = coo_y


class GrilleJeu(object):

    global pub_grille_jeu_Y
    global pub_grille_jeu_X

    def __init__(self):

        self.Case_Plateau_X = Float64MultiArray()
        self.Case_Plateau_Y = Float64MultiArray()
        self.ValideGrilleJeu = False
        rospy.Subscriber("tf",TFMessage,self.CalculGrille,queue_size = 1) #point origne du repere - SIFT

    def CalculGrille(self,data):

        if data.transforms[0].child_frame_id == "object_16" and self.ValideGrilleJeu == False:
            x_interet = data.transforms[0].transform.translation.x # Coo x du point d interet
            y_interet = data.transforms[0].transform.translation.y # Coo y du point d interet
            #A0
            A_0 = Case()
            A_0.ecrire(x_interet + MILLEU_CARRE,y_interet + MILLEU_CARRE)
            self.Case_Plateau_X.data.append(A_0.x)
            self.Case_Plateau_Y.data.append(A_0.y)
            #A1
            A_1 = Case()
            A_1.ecrire(x_interet + 2*MILLEU_CARRE,y_interet + MILLEU_CARRE)
            self.Case_Plateau_X.data.append(A_1.x)
            self.Case_Plateau_Y.data.append(A_1.y)
            #A2
            A_2 = Case()
            A_2.ecrire(x_interet + 3*MILLEU_CARRE,y_interet + MILLEU_CARRE)
            self.Case_Plateau_X.data.append(A_2.x)
            self.Case_Plateau_Y.data.append(A_2.y)
            #B0
            B_0 = Case()
            B_0.ecrire(x_interet + MILLEU_CARRE,y_interet + 2*MILLEU_CARRE)
            self.Case_Plateau_X.data.append(B_0.x)
            self.Case_Plateau_Y.data.append(B_0.y)
            #B1
            B_1 = Case()
            B_1.ecrire(x_interet + 2*MILLEU_CARRE,y_interet + 2*MILLEU_CARRE)
            self.Case_Plateau_X.data.append(B_1.x)
            self.Case_Plateau_Y.data.append(B_1.y)
            #B2
            B_2 = Case()
            B_2.ecrire(x_interet + 3*MILLEU_CARRE,y_interet + 2*MILLEU_CARRE)
            self.Case_Plateau_X.data.append(B_2.x)
            self.Case_Plateau_Y.data.append(B_2.y)
            #C0
            C_0 = Case()
            C_0.ecrire(x_interet + MILLEU_CARRE,y_interet + 3*MILLEU_CARRE)
            self.Case_Plateau_X.data.append(C_0.x)
            self.Case_Plateau_Y.data.append(C_0.y)
            #C1
            C_1 = Case()
            C_1.ecrire( x_interet + 2*MILLEU_CARRE,y_interet + 3*MILLEU_CARRE)
            self.Case_Plateau_X.data.append(C_1.x)
            self.Case_Plateau_Y.data.append(C_1.y)
            #C2
            C_2 = Case()
            C_2.ecrire(x_interet + 3*MILLEU_CARRE,y_interet + 3*MILLEU_CARRE)
            self.Case_Plateau_X.data.append(C_2.x)
            self.Case_Plateau_Y.data.append(C_2.y)

            self.ValideGrilleJeu = True
            pub_grille_jeu_X.publish(self.Case_Plateau_X)
            pub_grille_jeu_Y.publish(self.Case_Plateau_Y)


def main(arg):

    ic=GrilleJeu()

    rospy.init_node('Grille_Jeu', anonymous=True)
    print("JE SUIS LA")
    rospy.spin()


if __name__ == '__main__':
    main(sys.argv)
