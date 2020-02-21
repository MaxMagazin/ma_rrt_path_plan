"""
Path Planning Sample Code with RRT*

author: AtsushiSakai(@Atsushi_twi)
with edits of Maxim Yastremsky(@MaxMagazin)

"""

import random
import math
import copy
import numpy as np
import matplotlib.pyplot as plt
import time

import sys, select, termios, tty

class RRT():
    """
    Class for RRT Planning
    """

    def __init__(self, start, planDistance, obstacleList, expandDis=0.5, turnAngle=30, maxIter=1000, rrtTargets = None):

        self.start = Node(start[0], start[1], start[2])
        self.startYaw = start[2]

        self.planDistance = planDistance
        self.expandDis = expandDis
        self.turnAngle = math.radians(turnAngle)

        self.maxDepth = int(planDistance / expandDis)
        # print(self.maxDepth)

        self.maxIter = maxIter
        self.obstacleList = obstacleList
        self.rrtTargets = rrtTargets
        # self.end = Node(0, planDistance)

        self.aboveMaxDistance = 0
        self.belowMaxDistance = 0
        self.collisionHit = 0
        self.doubleNodeCount = 0

        self.savedRandoms = []

    def Planning(self, animation=False, interactive=False):
        self.nodeList = [self.start]
        self.leafNodes = []

        for i in range(self.maxIter):
            # rnd = self.get_random_point()
            rnd = self.get_random_point_from_target_list()

            # print "=====  random: {0},{1}".format(rnd[0], rnd[1]);

            nind = self.GetNearestListIndex(self.nodeList, rnd)

            nearestNode = self.nodeList[nind]
            # print("nearestNode: " + str(nearestNode))

            if (nearestNode.cost >= self.planDistance):
                # self.aboveMaxDistance += 1
                continue
            # self.belowMaxDistance += 1

            newNode = self.steerConstrained(rnd, nind)
            # newNode = self.steer(rnd, nind) #tests, delete

            # due to angle constraints it is possible that similar node is generated
            if newNode in self.nodeList:
                # self.doubleNodeCount += 1
                continue

            if self.__CollisionCheck(newNode, self.obstacleList):
                # nearinds = self.find_near_nodes(newNode)
                # newNode = self.choose_parent(newNode, nearinds)
                self.nodeList.append(newNode)
                # self.rewire(newNode, nearinds)

                if (newNode.cost >= self.planDistance):
                    # print("got a leaf " + str(newNode))
                    self.leafNodes.append(newNode)
            # else:
            #     self.collisionHit += 1

            if animation:
                self.DrawSample(rnd)

            if interactive:
                key = self.getKey()

                if (key == '\x03'): #CTRL+C
                    break

        # print "rrt.Planning(): aboveMaxDistance: {0}".format(self.aboveMaxDistance);
        # print "rrt.Planning(): belowMaxDistance: {0}".format(self.belowMaxDistance);
        # print "rrt.Planning(): doubleNodeCount: {0}".format(self.doubleNodeCount);
        # print "rrt.Planning(): collisionHit: {0}".format(self.collisionHit);
        # print "rrt.Planning(): nodeList length: {0}".format(len(self.nodeList));
        return self.nodeList, self.leafNodes

    def getKey(self):
      tty.setraw(sys.stdin.fileno())
      select.select([sys.stdin], [], [], 0)
      key = sys.stdin.read(1)
      termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
      return key

    def choose_parent(self, newNode, nearinds):
        if len(nearinds) == 0:
            return newNode

        dlist = []
        for i in nearinds:
            dx = newNode.x - self.nodeList[i].x
            dy = newNode.y - self.nodeList[i].y
            d = math.sqrt(dx ** 2 + dy ** 2)
            theta = math.atan2(dy, dx)
            if self.check_collision_extend(self.nodeList[i], theta, d):
                dlist.append(self.nodeList[i].cost + d)
            else:
                dlist.append(float("inf"))

        mincost = min(dlist)
        minind = nearinds[dlist.index(mincost)]

        if mincost == float("inf"):
            print("mincost is inf")
            return newNode

        newNode.cost = mincost
        newNode.parent = minind

        return newNode

    def steerConstrained(self, rnd, nind):
        # expand tree
        nearestNode = self.nodeList[nind]
        theta = math.atan2(rnd[1] - nearestNode.y, rnd[0] - nearestNode.x)

        # print "theta: {0}".format(math.degrees(theta));
        # print "nearestNode.yaw: {0}".format(math.degrees(nearestNode.yaw));

        # dynamic constraints
        angleChange = self.pi_2_pi(theta - nearestNode.yaw)

        # print "angleChange: {0}".format(math.degrees(angleChange));

        angle30degree = math.radians(30)

        if angleChange > angle30degree:
            angleChange = self.turnAngle
        elif angleChange >= -angle30degree:
            angleChange = 0
        else:
            angleChange = -self.turnAngle
        # print "angleChange2: {0}".format(math.degrees(angleChange));

        newNode = copy.deepcopy(nearestNode)
        newNode.yaw += angleChange
        newNode.x += self.expandDis * math.cos(newNode.yaw)
        newNode.y += self.expandDis * math.sin(newNode.yaw)

        newNode.cost += self.expandDis
        newNode.parent = nind

        # print "newNode: {0}".format(newNode)
        return newNode

    def pi_2_pi(self, angle):
        return (angle + math.pi) % (2*math.pi) - math.pi

    def steer(self, rnd, nind):
        # expand tree
        nearestNode = self.nodeList[nind]
        theta = math.atan2(rnd[1] - nearestNode.y, rnd[0] - nearestNode.x)
        newNode = copy.deepcopy(nearestNode)
        newNode.x += self.expandDis * math.cos(theta)
        newNode.y += self.expandDis * math.sin(theta)

        newNode.cost += self.expandDis
        newNode.parent = nind
        return newNode

    def get_random_point(self):

        randX = random.uniform(0, self.planDistance)
        randY = random.uniform(-self.planDistance, self.planDistance)
        rnd = [randX, randY]

        car_rot_mat = np.array([[math.cos(self.startYaw), -math.sin(self.startYaw)], [math.sin(self.startYaw), math.cos(self.startYaw)]])
        rotatedRnd = np.dot(car_rot_mat, rnd)

        rotatedRnd = [rotatedRnd[0] + self.start.x, rotatedRnd[1] + self.start.y]
        return rotatedRnd

    def get_random_point_from_target_list(self):

        maxTargetAroundDist = 3

        if not self.rrtTargets:
            return self.get_random_point()

        targetId = np.random.randint(len(self.rrtTargets))
        x, y, oSize = self.rrtTargets[targetId]

        # square idea
        # randX = random.uniform(-maxTargetAroundDist, maxTargetAroundDist)
        # randY = random.uniform(-maxTargetAroundDist, maxTargetAroundDist)
        # finalRnd = [x + randX, y + randY]

        # circle idea
        randAngle = random.uniform(0, 2 * math.pi)
        randDist = random.uniform(oSize, maxTargetAroundDist)
        finalRnd = [x + randDist * math.cos(randAngle), y + randDist * math.sin(randAngle)]

        return finalRnd

    def get_best_last_index(self):

        disglist = [self.calc_dist_to_goal(
            node.x, node.y) for node in self.nodeList]
        goalinds = [disglist.index(i) for i in disglist if i <= self.expandDis]
        #  print(goalinds)

        if len(goalinds) == 0:
            return None

        mincost = min([self.nodeList[i].cost for i in goalinds])
        for i in goalinds:
            if self.nodeList[i].cost == mincost:
                return i

        return None

    def gen_final_course(self, goalind):
        path = [[self.end.x, self.end.y]]
        while self.nodeList[goalind].parent is not None:
            node = self.nodeList[goalind]
            path.append([node.x, node.y])
            goalind = node.parent
        path.append([self.start.x, self.start.y])
        return path

    def calc_dist_to_goal(self, x, y):
        return np.linalg.norm([x - self.end.x, y - self.end.y])

    def find_near_nodes(self, newNode):
        nnode = len(self.nodeList)
        # r = 50.0 * math.sqrt((math.log(nnode) / nnode))
        r = self.expandDis * 3.0
        dlist = [(node.x - newNode.x) ** 2 +
                 (node.y - newNode.y) ** 2 for node in self.nodeList]
        nearinds = [dlist.index(i) for i in dlist if i <= r ** 2]
        # print "find_near_nodes, size: {0}".format(len(nearinds))
        return nearinds

    def rewire(self, newNode, nearinds):
        nnode = len(self.nodeList)
        for i in nearinds:
            nearNode = self.nodeList[i]

            dx = newNode.x - nearNode.x
            dy = newNode.y - nearNode.y
            d = math.sqrt(dx ** 2 + dy ** 2)

            scost = newNode.cost + d

            if nearNode.cost > scost:
                theta = math.atan2(dy, dx)
                if self.check_collision_extend(nearNode, theta, d):
                    nearNode.parent = nnode - 1
                    nearNode.cost = scost

    def check_collision_extend(self, nearNode, theta, d):

        tmpNode = copy.deepcopy(nearNode)

        for i in range(int(d / self.expandDis)):
            tmpNode.x += self.expandDis * math.cos(theta)
            tmpNode.y += self.expandDis * math.sin(theta)
            if not self.__CollisionCheck(tmpNode, self.obstacleList):
                return False

        return True

    def DrawSample(self, rnd=None):

        plt.clf()
        if rnd is not None:
            plt.plot(rnd[0], rnd[1], "^k")

        for node in self.nodeList:
            if node.parent is not None:
                plt.plot([node.x, self.nodeList[node.parent].x], [
                         node.y, self.nodeList[node.parent].y], "-g")

        # draw obstacles
        axes = plt.gca()
        for (ox, oy, size) in self.obstacleList:
            circle = plt.Circle((ox,oy), radius=size)
            axes.add_patch(circle)

        plt.plot(self.start.x, self.start.y, "xr")

        axes = plt.gca()
        xmin, xmax, ymin, ymax = -5, 25, -20, 20
        # plt.axis([-5, 45, -20, 20])
        axes.set_xlim([xmin,xmax])
        axes.set_ylim([ymin,ymax])
        # plt.axis("equal")
        plt.grid(True)
        plt.pause(0.001)

    def DrawGraph(self):
        # plt.clf()

        # draw obstacles
        ax = plt.gca()
        for (ox, oy, size) in self.obstacleList:
            circle = plt.Circle((ox,oy), radius=size)
            ax.add_patch(circle)

        for node in self.nodeList:
            if node.parent is not None:
                plt.plot([node.x, self.nodeList[node.parent].x], [
                         node.y, self.nodeList[node.parent].y], "-g")

        # plt.plot(self.start.x, self.start.y, "xr")
        # plt.plot(self.end.x, self.end.y, "xr")
        plt.axis([-5, 45, -20, 20])
        plt.axis("equal")
        plt.grid(True)
        plt.pause(0.01)

    def GetNearestListIndex(self, nodeList, rnd):
        dlist = [(node.x - rnd[0]) ** 2 + (node.y - rnd[1]) ** 2 for node in nodeList]
        minind = dlist.index(min(dlist))
        return minind

    def __CollisionCheck(self, node, obstacleList):
        for (ox, oy, size) in obstacleList:
            dx = ox - node.x
            dy = oy - node.y
            d = dx * dx + dy * dy
            if d <= size ** 2:
                return False  # collision
        return True  # safe

class Node():
    """
    RRT Node
    """

    def __init__(self, x, y, yaw):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.cost = 0.0
        self.parent = None

    def __str__(self):
        return str(round(self.x, 2)) + "," + str(round(self.y,2)) + "," + str(math.degrees(self.yaw)) + "," + str(self.cost)

    def __eq__(self, other):
        return self.x == other.x and self.y == other.y and self.yaw == other.yaw and self.cost == other.cost

    def __repr__(self):
        return str(self)


def main():
    print("Start rrt planning!")

    # ====Search Path with RRT====
    radius = 1
    obstacleList = [
        (1, -3, radius),
        (1, 3, radius),
        # (3, -3, radius),
        # (3, 3, radius),
        (6, -3, radius),
        (6, 3, radius),
        # (9, -2.8, radius),
        # (9, 3.2, radius),
        (12.5, -2.5, radius),
        (12, 4, radius),
        # (16, -2, radius),
        # (15, 5, radius),
        (20, -1, radius),
        (18.5, 6.5, radius),
        (24, 1, radius),
        (22, 8, radius)
    ]  # [x,y,size(radius)]

    start = [0.0, 0.0, math.radians(0.0)]
    planDistance = 10
    iterationNumber = 500
    rrtConeTargets = []

    for o in obstacleList:
        coneDist = math.sqrt((start[0] - o[0]) ** 2 + (start[1] - o[1]) ** 2)

        if coneDist > 10 and coneDist < 15:
            rrtConeTargets.append((o[0], o[1], radius))

    startTime = time.time()

    # Set Initial parameters
    rrt = RRT(start, planDistance, obstacleList=obstacleList, expandDis=1, maxIter=iterationNumber, rrtTargets = rrtConeTargets)
    rrt.Planning(True, True)
    # rrt.Planning()

    print "rrt.Planning(): {0} ms".format((time.time() - startTime) * 1000);
    print "nodesNumber/iteration: {0}/{1}".format(len(rrt.nodeList), iterationNumber)

    show_animation = True

    if show_animation:
        rrt.DrawGraph()
        # plt.plot([x for (x, y) in path], [y for (x, y) in path], '-r')
        plt.grid(True)
        plt.pause(0.01)  # Need for Mac
        plt.show()


if __name__ == '__main__':
    settings = termios.tcgetattr(sys.stdin)

    main()

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
