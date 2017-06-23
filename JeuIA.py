#!/usr/bin/env python
#--------------------------------------------------------------------------
# Tic Tac Toe game in Python
# Author: Mawuli Adzaku <mawuli@mawuli.me>
# Date: 20-05-2013
# Tested with Python 2.7
# TO RUN:
#         sudo chmod a+x tictactoe.py
#         ./tictactoe.py
# OR JUST RUN : python tictactoe.py
# @todo: parse user commands, log game scores, run game over telnet
# ---------------------------------------------------------------------------


# import modules
import random
import sys
import copy
import rospy
from std_msgs.msg import *

pub = rospy.Publisher('Case_Jeu',Int8MultiArray,queue_size=1)


class Game:
    global pub
    def __init__(self):

        self.u8t_jeu=Int8MultiArray()
        self.board = [' '] * 9
        self.player_name = ''
        self.player_marker = ''
        self.bot_name = 'TBot'
        self.bot_marker = ''
        self.winning_combos = (
        [6, 7, 8], [3, 4, 5], [0, 1, 2], [0, 3, 6], [1, 4, 7], [2, 5, 8],
        [0, 4, 8], [2, 4, 6],
    )
        self.corners = [0,2,6,8]
        self.sides = [1,3,5,7]
        self.middle = 4

        self.form = '''
           \t| %s | %s | %s |
           \t-------------
           \t| %s | %s | %s |
           \t-------------
           \t| %s | %s | %s |
           '''

    def print_board(self,board = None):
        "Display board on screen"
        if board is None:
            print self.form % tuple(self.board[6:9] + self.board[3:6] + self.board[0:3])
        else:
            # when the game starts, display numbers on all the grids
            print self.form % tuple(board[6:9] + board[3:6] + board[0:3])

    def get_marker(self):
        return ('X', 'Y')


    def quit_game(self):
        "exits game"
        self.print_board
        print "\n\t Thanks for playing :-) \n\t Come play again soon!\n"
        sys.exit()

    def is_winner(self, board, marker):
        "check if this marker will win the game"
        # order of checks:
        #   1. across the horizontal top
        #   2. across the horizontal middle
        #   3. across the horizontal bottom
        #   4. across the vertical left
        #   5. across the vertical middle
        #   6. across the vertical right
        #   7. across first diagonal
        #   8. across second diagonal
        for combo in self.winning_combos:
            if (board[combo[0]] == board[combo[1]] == board[combo[2]] == marker):
                return True
        return False

    def get_bot_move(self):
        '''
        find the best space on the board for the bot. Objective
        is to find a winning move, a blocking move or an equalizer move.
        Bot must always win
        '''

        # check if bot can win in the next move
        for i in range(0,len(self.board)):
            board_copy = copy.deepcopy(self.board)
            if self.is_space_free(board_copy, i):
                self.make_move(board_copy,i,self.bot_marker)
                if self.is_winner(board_copy, self.bot_marker):
                    return i

        # check if player could win on his next move
        for i in range(0,len(self.board)):
            board_copy = copy.deepcopy(self.board)
            if self.is_space_free(board_copy, i):
                self.make_move(board_copy,i,self.player_marker)
                if self.is_winner(board_copy, self.player_marker):
                    return i

        # check for space in the corners, and take it
        move = self.choose_random_move(self.corners)
        if move != None:
            return move

        # If the middle is free, take it
        if self.is_space_free(self.board,self.middle):
            return self.middle


        # else, take one free space on the sides
        return self.choose_random_move(self.sides)

    def is_space_free(self, board, index):
        "checks for free space of the board"
        # print "SPACE %s is taken" % index
        return board[index] == ' '

    def is_board_full(self):
        "checks if the board is full"
        for i in range(1,9):
            if self.is_space_free(self.board, i):
                return False
        return True

    def make_move(self,board,index,move):
        board[index] =  move

    def choose_random_move(self, move_list):
        possible_winning_moves = []
        for index in move_list:
            if self.is_space_free(self.board, index):
                possible_winning_moves.append(index)
                if len(possible_winning_moves) != 0:
                    return random.choice(possible_winning_moves)
                else:
                    return None

    def start_game(self):
       self.print_board(range(1,10))

       # get user's preferred marker
       self.player_marker, self.bot_marker = self.get_marker()

       # randomly decide who can play first
       if random.randint(0,1) == 0:
           self.enter_game_loop('b')
       else:
           # now, enter the main game loop
           self.enter_game_loop('h')


    def enter_game_loop(self,turn):
       "starts the main game loop"
       is_running = True
       player = turn #h for human, b for bot
       while is_running:
           if player == 'h' :

               bot_move = self.get_bot_move()

               '''AJOUT DE CODE'''
               self.u8t_jeu.data.append(bot_move)
               '''FIN AJOUT DE CODE'''

               self.make_move(self.board, bot_move, self.player_marker)
               if(self.is_winner(self.board, self.bot_marker)):
                   self.print_board()
                   print "\n\tCONGRATULATIONS %s, YOU HAVE WON THE GAME!!! \\tn" % self.bot_name
                   is_running = False
                   #break
               else:
                   if self.is_board_full():
                       self.print_board()
                       print "\n\t-- Match Draw --\t\n"
                       is_running = False
                       #break
                   else:
                       self.print_board()
                       player = 'b'
           # bot's turn to play
           else:

              bot_move =  self.get_bot_move()

              '''AJOUT DE CODE'''
              self.u8t_jeu.data.append(bot_move)
              '''FIN AJOUT DE CODE'''

              self.make_move(self.board, bot_move, self.bot_marker)
              if (self.is_winner(self.board, self.bot_marker)):
                  self.print_board()
                  print "\n\t%s HAS WON!!!!\t\n" % self.bot_name
                  #self.incr_score(self.bot_name)
                  is_running = False
                  break
              else:
                  if self.is_board_full():
                      self.print_board()
                      print "\n\t -- Match Draw -- \n\t"
                      is_running = False
                      #break
                  else:
                      self.print_board()
                      player = 'h'

       # when you break out of the loop, end the game
       '''AJOUT DE CODE'''
       print("JEU IA")
       pub.publish(self.u8t_jeu)
       '''FIN AJOUT DE CODE'''
       self.end_game()

    def end_game(self):
       play_again = raw_input("Would you like to play again? (y/n): ").lower()
       if play_again == 'y':
           self.__init__() # necessary for re-initialization of the board etc
           self.start_game()
       else:
           print "\n\t-- GAME OVER!!!--\n\t"
           self.quit_game()




def main(arg):

    rospy.init_node('Jeu',anonymous=True)

    TicTacToe = Game()
    TicTacToe.start_game() #Debut du jeu de TicTacToe


    rospy.spin()

if __name__ == "__main__":
     main(sys.argv)
