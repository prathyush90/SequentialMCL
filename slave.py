import math
import random
from copy import deepcopy
class Particle(object):

    def __init__(self, x, y, orientation, geofence):
        self.x = x
        self.y = y
        self.orientation = orientation
        self.geofence = geofence
        self.x = self.adjustCordinates(self.x, 0)
        self.y = self.adjustCordinates(self.y, 1)
        self.weight = 1

    def setNoise(self,forward_noise, turn_noise):
        self.forward_noise = forward_noise
        self.turn_noise    = turn_noise

    def getX(self):
        return self.x

    def getY(self):
        return self.y

    def getOrientation(self):
        return self.orientation*math.pi/180

    def getWeight(self):
        return self.weight


    def setWeight(self, val):
        self.weight = val


    def adjustCordinates(self, value, index):
        adjustedValue = value
        if(value< self.geofence[index][0]):
            adjustedValue = self.geofence[index][1] - self.geofence[index][0] + value
        elif(value > self.geofence[index][1]):
            adjustedValue = self.geofence[index][0] + value - self.geofence[index][1]

        return adjustedValue
    def move(self,heading,noise,distance, isRobot):

        if(isRobot):
            turn = (heading + (random.random() * 2 - 1.0) * noise)* math.pi / 180.0
        else:
            turn = ((self.orientation+heading) + (random.random() * 2 - 1.0) * noise)* math.pi / 180.0

        d    = distance + (random.random()*2 - 1.0)*self.forward_noise
        self.x = self.x + d * math.sin(turn)
        self.y = self.y + d * math.cos(turn)
        self.x = self.adjustCordinates(self.x, 0)
        self.y = self.adjustCordinates(self.y, 1)
        self.orientation = ((turn*180/math.pi)+360)%360

        return deepcopy(self)

    def gaussian(self, mu, sigma, predictedValue):
        num = -0.5 * pow(-1*(mu -predictedValue)/sigma,2)
        num = math.exp(num)
        den = math.sqrt(2 * math.pi)*sigma+0.00000000000000000000000000005
        return num/den

    def measurementProbability(self, robotPos):
        rmse  = math.sqrt(math.pow(self.x - robotPos[0], 2) + math.pow(self.y - robotPos[1], 2))
        sigma = 3.0
        gauss = self.gaussian(0,sigma, rmse)
        prob  = math.sqrt(gauss)
        self.weight *= prob
        return prob

    def measurementProbabilityForDistribution(self, given, observed):
        prob = 1
        for i in range(0, len(given)):
            gauss = self.gaussian(given[i], 8*pow(10, -2), observed[i])
            prob  *= math.sqrt(gauss)
        self.weight *= prob
        return prob

    def measurementProbabilityPseudo(self, robotPos, sigma):
        robotPos[0] = robotPos[0]
        robotPos[1] = robotPos[1]
        rmse  = math.sqrt(math.pow(self.x - robotPos[0], 2) + math.pow(self.y - robotPos[1], 2))
        sigma = 2.0
        gauss = self.gaussian(0,2.0, rmse)
        prob  = math.sqrt(gauss)
        return prob

    def returnRmse(self, robotPos):
        rmse = math.sqrt(math.pow(self.x - robotPos[0], 2) + math.pow(self.y - robotPos[1], 2))
        return rmse