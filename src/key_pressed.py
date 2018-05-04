#!/usr/bin/env python

# import keyboard #Using module keyboard
# while True:#making a loop
#     try: #used try so that if user pressed other than the given key error will not be shown
#         if keyboard.is_pressed('q'):#if key 'q' is pressed 
#             print('You Pressed A Key!')
#         else:
#             pass
#     except:
#         print 'break'
#         break #if user pressed a key other than the given key the loop will break

# import sys, select, tty, termios

# # BEGIN TERMIOS
# old_attr = termios.tcgetattr(sys.stdin)
# tty.setcbreak(sys.stdin.fileno())
# # END TERMIOS
# print "Publishing keystrokes. Press Ctrl-C to exit..."
# while True:
#     # BEGIN SELECT
#     if select.select([sys.stdin], [], [], 0)[0] == [sys.stdin]:
#         print sys.stdin.read(1)
#     # END SELECT
# # BEGIN TERMIOS_END
# termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_attr)
# END TERMIOS_END

# import pygame
# while True:
#     pressed = pygame.key.get_pressed()
#     if pressed[pygame.K_w]:
#        print("w is pressed")
#     if pressed[pygame.K_s]:
#        print("s is pressed")

import cv2

cv2.namedWindow('kalala')
while True:
    key = cv2.waitKey(1)
    if key == ord('a'):
        print key