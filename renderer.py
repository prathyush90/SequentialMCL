import pygame
import sys
import pygame.locals as settings
from slavemanager import slaveManager as Manager
import math
import random
FPS=30
WINDOWWIDTH = 500
WINDOWHEIGHT = 500
COLOR = { "white": (255, 255, 255),
          "black": (0, 0, 0),
          "green": (0, 255, 0),
          "blue": (0, 0, 255),
          "red": (255, 0, 0),
          "purple": (128, 0, 128)
        }

initialheading = 0
pygame.init()
window = pygame.display.set_mode((WINDOWWIDTH, WINDOWHEIGHT), 0, 32)
particlemanager = Manager([[0, WINDOWWIDTH], [0, WINDOWHEIGHT]], 500)
particlemanager.initSlaves()
fpsClock = pygame.time.Clock()

def test_end(event):
    if event.type == settings.QUIT:
        pygame.quit()
        sys.exit()

def clear():
    """Fill the background white"""
    window.fill(COLOR["white"])

def dick(pos_x, pos_y, orientation):
    orientation = (orientation * math.pi)/180.0
    return [(pos_x, pos_y), (pos_x + 25 * math.sin(orientation), pos_y + 25 * math.cos(orientation))]

def convertCoordinates(x, y):
    return (int(x), int(WINDOWHEIGHT - y))

def renderOnScreen():
    robot_coords     = particlemanager.getRobotCoords()
    predicted_coords = particlemanager.getUserExpectedCoords()
    predicted_orie   = particlemanager.getUserExpectedHeading()
    #draw robot
    pygame.draw.circle(window, COLOR["blue"], convertCoordinates(robot_coords[0], robot_coords[1]), 7)
    #draw particles
    slaves = particlemanager.getParticles()
    for slave in slaves:
        pygame.draw.circle(window, COLOR["green"], convertCoordinates(slave.getX(), slave.getY()), 7)
    #draw expected point
    pygame.draw.circle(window, COLOR["red"], convertCoordinates(predicted_coords[0],predicted_coords[1]), 7)
    #draw real orientation
    pygame.draw.line(window, COLOR["purple"], *[convertCoordinates(pos[0], pos[1]) for pos in dick(robot_coords[0], robot_coords[1], initialheading)])

    #draw leftmost orientation
    pygame.draw.line(window, COLOR["black"],
                     *[convertCoordinates(pos[0], pos[1]) for pos in dick(predicted_coords[0], predicted_coords[1], predicted_orie)])

    fpsClock.tick(FPS)
    pygame.display.update()

while True:
    if(particlemanager.isBusy):
        continue
    for event in pygame.event.get():
        test_end(event)
    clear()
    key_pressed = pygame.key.get_pressed()
    #if forward key
    if key_pressed[settings.K_UP]:
        particlemanager.move(initialheading, 0.5, 0)
    #if left key pressed
    if key_pressed[settings.K_LEFT]:
        initialheading -= 5
        initialheading += 360
        initialheading %= 360
        particlemanager.move(initialheading, 0.0, -5)

    #if right key pressed
    if key_pressed[settings.K_RIGHT]:
        initialheading += 5
        initialheading += 360
        initialheading %= 360
        particlemanager.move(initialheading, 0.0, 5)

    if(key_pressed[settings.K_w]):
        #random movement
        copyheading = initialheading
        initialheading += (random.random()*90+90)
        initialheading += 360
        initialheading %= 360
        particlemanager.move(initialheading, 0.5, 0)
        #initialheading = copyheading
    renderOnScreen()

