import random
from slave import Particle
import math
from copy import deepcopy
from numpy import array
from sklearn.decomposition import PCA
from copy import deepcopy
import numpy as np
from sklearn.neighbors import KNeighborsRegressor

class slaveManager(object):
    def __init__(self, geofence,particlSize):
        self.geofence       = geofence
        self.particleSize   = particlSize
        self.turn_noise     = 45.0 * (math.pi/180.0)
        self.forward_noise  = 0.01
        self.posx = 0.0
        self.posy = 0.0
        self.previousx = 0.0
        self.previousy = 0.0
        self.count = 0
        self.THRESHOLD = 5
        self.avoidResample = False
        self.previoussmoothing = 0.8
        self.isBusy = False
        self.slaveDiff = []
        #self.placeMagnets()
        self.walkAroundTheGrid()
        self.trainKDTree()
        self.initialLocation = True
        self.previousHeading = 0;






    def getIsBusy(self):
        return self.isBusy

    def getRobotCoords(self):
        return [self.robot.getX(), self.robot.getY()]

    def getUserExpectedCoords(self):
        return [self.posx, self.posy]

    def getUserExpectedHeading(self):
        return self.heading

    def initSlaves(self):
        self.slaves = []
        robot_x = random.random()*(self.geofence[0][1] - self.geofence[0][0]) + self.geofence[0][0]
        robot_y = random.random()*(self.geofence[1][1] - self.geofence[1][0]) + self.geofence[1][0]
        self.robot  = Particle(robot_x, robot_y, random.random()*math.pi*2, self.geofence)
        self.robot.setNoise(0, 0)
        for number in range(self.particleSize):

            x           = random.uniform(0,1)*(self.geofence[0][1] - self.geofence[0][0]) + self.geofence[0][0]
            y           = random.uniform(0,1)*(self.geofence[1][1] - self.geofence[1][0]) + self.geofence[1][0]
            orientation = random.random()*math.pi*2
            particle    = Particle(x, y, orientation, self.geofence)
            particle.setNoise(self.forward_noise, self.turn_noise)
            self.slaves.append(particle)
        self.previousslaves = deepcopy(self.slaves)


        self.posx, self.posy, self.heading = self.getUserPosition([1]*self.particleSize)
        self.heading = random.random()*360

    def move(self,turn_steer,distance, turn_deg):
        turn = turn_deg
        print(turn)
        self.previousHeading = turn_steer;
        for number in range(self.particleSize):
            particle = self.slaves[number]
            #particle.setNoise(self.forward_noise, self.turn_noise)


            headingMean = random.random()*360
            headingStd = 10


            particle = particle.move(turn,headingStd, distance, False)
            self.slaves[number] = particle

        self.robot = self.robot.move(turn_steer,0, distance, True)
        weights = []


        Y = np.array(self.getStrengthAtPoint(self.robot.getX(), self.robot.getY()))
        # Y = Y.reshape(-1, 2)
        # Y = self.neigh.predict(Y)
        val = [self.robot.getX(), self.robot.getY()]
        #pred_robot = self.sigmoid(poww)
        # num = random.random()*10
        # if(num < 2):
        #     self.initialLocation = False
        #     #print("beacon too")
        for number in range(self.particleSize):
            particle = self.slaves[number]

            # Y_Particle = np.array([particle.getX(), particle.getY()])
            # Y_Particle = Y_Particle.reshape(-1, 2)
            # Y_Particle_array = self.neigh.kneighbors(Y_Particle)
            # val_for_particle = self.returnValue(Y_Particle_array, Y_Particle)
            # val_particle = [particle.getX(), particle.getY()]
            #pred = self.sigmoid(powww)
            weight   = 1
            if(not self.initialLocation):
                weight   = particle.measurementProbability([self.robot.getX(), self.robot.getY()])

            #weight   *= particle.measurementProbabilityForDistribution(Y, val_for_particle)
            weight = particle.measurementProbability([self.robot.getX(), self.robot.getY()])
            weights.append(weight)
        self.initialLocation = True
        #self.previousx,self.previousy = self.posx*(1-self.previoussmoothing) + self.previoussmoothing*self.previousx , self.posy*(1-self.previoussmoothing)+self.previoussmoothing*self.previousy
        self.posx, self.posy, self.heading = self.getUserPosition(weights)
        print(self.heading)
        self.normalizeParticleWeights()
        self.checkNeedForResample()
        min_weight = min(weights)
        max_weigth = max(weights)
        # #print("-----")
        # print(max_weigth)
        # print(min_weight)
        # #print("-----")
        #check      = min_weight/max_weigth
        # if(check >= 0.1):
        #     self.avoidResample = True

        self.resample(weights)
        #self.getHeading()
        self.previousx, self.previousy = self.posx, self.posy
        self.count += 1

        #self.calculateVariance()

        # if(abs(self.heading - turn) > 90):
        #     self.heading += 180
        self.isBusy = False

    def returnValue(self, Y_particle_array, particle):

        particle = particle[0]

        pred = self.getStrengthAtPoint(particle[0], particle[1])
        distances          = Y_particle_array[0][0]
        distances = [1.0/x for x in distances]
        #print(distances)
        ##print(distances)
        distances          /= sum(distances)
        #print(distances)

        neigborIndexes  = Y_particle_array[1][0]
        x = 0;
        y = 0;
        for i,index in enumerate(neigborIndexes):
            predicted = self.Y[index]
            x += distances[i]*predicted[0]
            y += distances[i]*predicted[1]

        rmse = pow(x - pred[0], 2)+pow(y-pred[1], 2)
        #print(rmse)

        return [x,y]


        ##print(Y_particle_array)

    def calculateOrientationDifference(self,x,y,prevx,prevY):
        ydiff = y - prevY
        xdiff = x - prevx
        angle = math.atan2(xdiff, ydiff)
        angle = angle*180/math.pi
        angle += 360
        angle %= 360
        return angle

    def needToSpreadParticles(self):
        self.isBusy = True
        for i in range(self.particleSize):
            x = self.posx + (random.uniform(0,1)*(self.geofence[0][1] - self.geofence[0][0])/2.0)
            y = self.posy + (random.uniform(0,1)*(self.geofence[1][1] - self.geofence[1][0]) + self.geofence[1][0]/2.0)
            orientation = random.random() * math.pi * 2
            particle = Particle(x, y, orientation, self.geofence)
            particle.setNoise(self.forward_noise, self.turn_noise)
            self.slaves[i] = particle
        self.posx, self.posy, self.heading = self.getUserPosition([1] * self.particleSize)
        self.heading = self.getUserEstimatedOrientation()

    def getUserPosition(self,weights):
        xmean, ymean, zmean, sumWeight, count = 0.0, 0.0, 0.0, 0.0, 0

        # for i in range(self.particleSize):
        #     sumWeight += weights[i]

        for i in range(self.particleSize):
            sumWeight += self.slaves[i].getWeight()

        if(sumWeight == 0):
            sumWeight = 0.000000000000000000001

        # if(sumWeight == 0):
        #     sumWeight += 0.0005
        #     #self.avoidResample = True
        # else:
        #     #self.avoidResample = False
        orientation = 0
        index_max = np.argmax(np.array(weights))
        first_particle_weight = (self.slaves[0].getWeight()/sumWeight)
        for index,particle in enumerate(self.slaves):
            weight           = particle.getWeight()
            normalizedWeight = weight/sumWeight
            count += normalizedWeight
            xmean += normalizedWeight * particle.getX()
            ymean += normalizedWeight * particle.getY()
            #orientation += (normalizedWeight) * particle.getOrientation()
            # (((p[i].orientation - p[0].orientation + pi) % (2.0 * pi))
            #  + p[0].orientation - pi)
            orientation += ((((normalizedWeight) * particle.getOrientation() - first_particle_weight*self.slaves[0].getOrientation() + math.pi)% (2.0 * math.pi))+(first_particle_weight*self.slaves[0].getOrientation()-math.pi))

        # if(count < 0.1):
        #     #self.avoidResample = True
        #     #self.needToSpreadParticles()
        #     return self.posx,self.posy
        if(count == 0):
            count = 0.00000000000000000001
        xmean /= count
        ymean /= count
        orientation /= count
        orientation = orientation*180/math.pi
        return xmean, ymean, orientation




    def resample(self, weights):
        if(self.avoidResample):
            return
        rand_index  = random.randint(0, self.particleSize-1)
        beta        = 0.0
        max_weight  = max(weights)
        new_array = []
        self.previousslaves = deepcopy(self.slaves)
        for i,particle in enumerate(self.slaves):
            beta += random.random() * 2.0 * max_weight
            while beta >= weights[rand_index]:
                beta -= weights[rand_index]
                rand_index = (rand_index + 1) % self.particleSize


            particle = deepcopy(self.slaves[rand_index])
            particle.setWeight(1)

            new_array.append(particle)
        self.slaves = new_array

    def getUserEstimatedOrientation(self, turn, distance):
        particlesCopy = deepcopy(self.slaves)
        robot         = deepcopy(self.robot)
        for number in range(self.particleSize):
            particle = particlesCopy[number]
            head     = random.random()*(360)
            particle = particle.move(head, 0, distance)
            particlesCopy[number] = particle
        robot = robot.move(turn, 0,distance)
        weights = []


        for number in range(self.particleSize):
            particle = particlesCopy[number]
            weight   = particle.measurementProbabilityPseudo([robot.getX(), robot.getY()], 0+1)
            weights.append(weight)

        headinVals = []
        min_weight = min(weights)
        max_weight = max(weights)
        if(min_weight ==  max_weight):
            #print("exception")
            for number in range(self.particleSize):
                headinVals.append(random.random()*360)
            return headinVals, np.mean(headinVals), np.std(headinVals)
        for number in range(1000):
            rand_index = random.randint(0, self.particleSize - 1)
            beta = 0.0
            max_weight = max(weights)
            new_array = []
            for number in range(0,5):
                beta += random.random() * 2.0 * max_weight
                while beta >= weights[rand_index]:
                    beta -= weights[rand_index]
                    rand_index = (rand_index + 1) % self.particleSize
                particle = deepcopy(particlesCopy[rand_index])
                new_array.append(particle.getOrientation())
            headinVals.append(np.mean(new_array))



        return headinVals,np.mean(headinVals),np.std(headinVals)





    # def calculateVariance(self):
    #     upwardpoint     = None
    #     downwardpoint   = None
    #     maxupwarddistance =0
    #     maxdownwarddistance = 0
    #
    #     for index,particle in enumerate(self.slaves):
    #         if(index ==0):
    #             upwardpoint   = [particle.getX(), particle.getY()]
    #             maxupwarddistance = math.sqrt(pow((particle.getX()-self.posx), 2)+pow((particle.getY()-self.posy), 2))
    #         else:
    #             #opp sides ((y1-y2)*(ax-x1)+(x2-x1)*(ay-y1))((y1-y2)*(bx-x1)+(x2-x1)*(by-y1))
    #             ax = particle.getX()
    #             ay = particle.getY()
    #             bx = upwardpoint[0]
    #             by = upwardpoint[1]
    #             y1 = 0
    #             x1 = 0
    #             y2 = self.posy
    #             x2 = self.posx
    #             value = ((y1-y2)*(ax-x1)+(x2-x1)*(ay-y1))*((y1-y2)*(bx-x1)+(x2-x1)*(by-y1))
    #             if(value > 0):
    #                 #same side
    #                 distance = math.sqrt(pow((particle.getX()-self.posx), 2)+pow((particle.getY()-self.posy), 2))
    #                 if(distance > maxupwarddistance):
    #                     maxupwarddistance = distance
    #                     upwardpoint = [particle.getX(), particle.getY()]
    #             else:
    #                 distance = math.sqrt(
    #                     pow((particle.getX() - self.posx), 2) + pow((particle.getY() - self.posy), 2))
    #                 if(downwardpoint):
    #
    #                     if (distance > maxdownwarddistance):
    #                         maxdownwarddistance = distance
    #                         downwardpoint = [particle.getX(), particle.getY()]
    #                 else:
    #
    #                     maxdownwarddistance = distance
    #                     downwardpoint = [particle.getX(), particle.getY()]
    #
    #     topangle =0
    #     bottomangle=0
    #     if(upwardpoint):
    #         topangle    = ((math.atan2(upwardpoint[0], upwardpoint[1])*180/math.pi)+360) % 360
    #
    #     if(downwardpoint):
    #         bottomangle = ((math.atan2(downwardpoint[0], downwardpoint[1])*180/math.pi)+360) % 360
    #
    #     diff        = (abs(bottomangle - topangle)+5)/2.0
    #     self.turn_noise = diff * (math.pi/180.0)





    def getParticles(self):
        return self.slaves

    def returnTurnNoise(self):
        return self.turn_noise

    def checkNeedForResample(self):
        sumWeights = 0
        for i in range(self.particleSize):
            slave = self.slaves[i]
            val   = slave.getWeight()
            val   = pow(val, 2)
            sumWeights += val
        if(sumWeights == 0):
            sumWeights += 0.000000000000000001
        invWeight = 1/sumWeights
        if(invWeight < self.particleSize/2):
            self.avoidResample = False
            #print("resampling")
        else:
            self.avoidResample = True


        return

    def normalizeParticleWeights(self):
        sum = 0
        for i in range(self.particleSize):
            slave = self.slaves[i]
            sum += slave.getWeight()
        if(sum == 0):
            sum = 0.000000000000000001
        for i in range(self.particleSize):
            slave = self.slaves[i]
            slave.setWeight(slave.getWeight()/sum)
            self.slaves[i] = deepcopy(slave)

        return

    def walkAroundTheGrid(self):
       minx = self.geofence[0][0]
       maxx = self.geofence[0][1]
       miny = self.geofence[1][0]
       maxy = self.geofence[1][1]
       self.X = []
       self.Y = []
       for y in range(miny, maxy):
           for x in range(minx, maxx):
               inp = [x, y]
               self.X.append(inp)
               magAtPoint = self.getStrengthAtPoint(x, y)
               self.Y.append(magAtPoint)

       for x in range(minx, maxx):
           for y in range(miny, maxy):
               inp = [x, y]
               self.X.append(inp)
               magAtPoint = self.getStrengthAtPoint(x, y)
               self.Y.append(magAtPoint)

       self.X = np.array(self.X)
       self.Y = np.array(self.Y)
       self.Y = self.Y.reshape(-1, 2)
       ##print(self.Y)

    def trainKDTree(self):
        self.neigh = KNeighborsRegressor(n_neighbors=2, algorithm='kd_tree')
        #print(self.Y.shape)
        self.neigh.fit(self.X, self.Y)


    def sigmoid(self, x):
        return 1/(1+math.exp(-x))

    def placeMagnets(self):
        self.magnetsList = []
        for i in range(0, 500):
            mag_x = random.random() * (self.geofence[0][1] - self.geofence[0][0]) + self.geofence[0][0]
            mag_y = random.random() * (self.geofence[1][1] - self.geofence[1][0]) + self.geofence[1][0]
            magstrength_x = random.random()*5
            magstrength_y = random.random()*5
            mag_obj = {}
            mag_obj['x']  = mag_x
            mag_obj['y']  = mag_y
            mag_obj['strengthx'] = magstrength_x
            mag_obj['strengthy'] = magstrength_y
            self.magnetsList.append(mag_obj)

    def getStrengthAtPoint(self, x, y):
        # mag_x = 0
        # mag_y = 0
        # for magnet in self.magnetsList:
        #     diff_x = abs(magnet['x'] - x)
        #     diff_y = abs(magnet['y'] - y)
        #     mag_x  += (magnet['strengthx'] / diff_x)
        #     mag_y  += (magnet['strengthy'] / diff_y)
        # x /= 2
        # y /= 2

        x += 0.0001
        y += 0.0001

        mag_x = 100.0/x
        mag_y = 100.0/y

        return [x, y]

    def getHeading(self):
        if(self.previousx and self.previousy):
            dx = self.posx - self.previousx
            dy = self.posy - self.previousy
            self.heading = math.atan2(dx, dy)*180/math.pi
            self.heading += 360
            self.heading %= 360
        else:
            self.heading = random.random()*360







