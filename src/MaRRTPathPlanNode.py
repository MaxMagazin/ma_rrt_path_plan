"""
RRT Path Planning with multiple remote goals.

author: Maxim Yastremsky(@MaxMagazin)
based on the work of AtsushiSakai(@Atsushi_twi)

"""
import rospy
import csv

import ma_rrt

from vehicle_msgs.msg import TrackCone, Track, Command, Waypoint, WaypointsArray

from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from geometry_msgs.msg import Point
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from nav_msgs.msg import Path

# For odometry message
from tf.transformations import euler_from_quaternion

from scipy.spatial import Delaunay

# Matrix/Array library
import numpy as np
import time, math

class MaRRTPathPlanNode:
    # All variables, placed here are static

    def __init__(self):
        # Get all parameters from launch file
        self.shouldPublishWaypoints = rospy.get_param('~publishWaypoints', True)
        self.shouldPublishPredefined = rospy.get_param('~publishPredefined', False)

        if rospy.has_param('~path'):
            self.path = rospy.get_param('~path')

        if rospy.has_param('~filename'):
            self.filename = rospy.get_param('~filename')

        if rospy.has_param('~odom_topic'):
            self.odometry_topic = rospy.get_param('~odom_topic')
        else:
            self.odometry_topic = "/odometry"

        if rospy.has_param('~world_frame'):
            self.world_frame = rospy.get_param('~world_frame')
        else:
            self.world_frame = "world"

        waypointsFrequency = rospy.get_param('~desiredWaypointsFrequency', 5)
        self.waypointsPublishInterval = 1.0 / waypointsFrequency
        self.lastPublishWaypointsTime = 0

        # All Subs and pubs
        rospy.Subscriber("/track", Track, self.mapCallback)
        rospy.Subscriber(self.odometry_topic, Odometry, self.odometryCallback)
        rospy.Subscriber("/dvsim/cmd", Command, self.carSensorsCallback)

        # Create publishers
        self.waypointsPub = rospy.Publisher("/waypoints", WaypointsArray, queue_size=0)

        # visuals
        self.treeVisualPub = rospy.Publisher("/visual/tree_marker_array", MarkerArray, queue_size=0)
        self.bestBranchVisualPub = rospy.Publisher("/visual/best_tree_branch", Marker, queue_size=1)
        self.filteredBranchVisualPub = rospy.Publisher("/visual/filtered_tree_branch", Marker, queue_size=1)
        self.delaunayLinesVisualPub = rospy.Publisher("/visual/delaunay_lines", Marker, queue_size=1)
        self.waypointsVisualPub = rospy.Publisher("/visual/waypoints", MarkerArray, queue_size=1)

        self.carPosX = 0.0
        self.carPosY = 0.0
        self.carPosYaw = 0.0

        self.map = []
        self.savedWaypoints = []
        self.preliminaryLoopClosure = False
        self.loopClosure = False

        self.rrt = None

        if self.shouldPublishPredefined:
            with open(self.path + self.filename) as csv_file:
                csv_reader = csv.reader(csv_file, delimiter=',')

                for row in csv_reader:
                    self.savedWaypoints.append((float(row[0]), float(row[1])))

                # print("predefinedWaypoints:", self.savedWaypoints)
            self.preliminaryLoopClosure = True
            self.loopClosure = True

        self.filteredBestBranch = []
        self.discardAmount = 0

        print("MaRRTPathPlanNode Constructor has been called")

    def __del__(self):
        print('MaRRTPathPlanNode: Destructor called.')

    def odometryCallback(self, odometry):
        # rospy.loginfo("odometryCallback")

        # start = time.time()
        orientation_q = odometry.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, yaw)  = euler_from_quaternion(orientation_list)

        self.carPosX = odometry.pose.pose.position.x
        self.carPosY = odometry.pose.pose.position.y
        self.carPosYaw = yaw
        #print "Estimated processing odometry callback: {0} ms".format((time.time() - start)*1000)

    def carSensorsCallback(self, command):
        # rospy.loginfo("carSensorsCallback")

        # start = time.time()
        # The ackermann angle [rad]
        self.steerAngle = math.degrees((command.theta_l + command.theta_r) / 2.0)

        #print "Estimated processing map callback: {0} ms".format((time.time() - start)*1000);

    def mapCallback(self, track):
        self.map = track.cones

    def sampleTree(self):
        # sampleTreeStartTime = time.time()
        if self.loopClosure and len(self.savedWaypoints) > 0:
            # print("Publish savedWaypoints/predifined waypoints, return")
            self.publishWaypoints()
            return

        # print("------- sampleTree() -------");

        if not self.map:
            # print "sampleTree(): map is still empty, return"
            return

        # print "map size: {0}".format(len(self.map.cones));

        frontConesDist = 12
        frontCones = self.getFrontConeObstacles(self.map, frontConesDist)
        # frontCones = [] # empty for tests

        # print "frontCones size: {0}".format(len(frontCones));
        # print(frontCones)

        coneObstacleSize = 1.1
        coneObstacleList = []
        rrtConeTargets = []
        coneTargetsDistRatio = 0.5
        for cone in frontCones:
            coneObstacleList.append((cone.x, cone.y, coneObstacleSize))

            coneDist = self.dist(self.carPosX, self.carPosY, cone.x, cone.y)

            if coneDist > frontConesDist * coneTargetsDistRatio:
                rrtConeTargets.append((cone.x, cone.y, coneObstacleSize))

        # Set Initial parameters
        start = [self.carPosX, self.carPosY, self.carPosYaw]
        iterationNumber = 1000
        planDistance = 12
        expandDistance = 1.0
        expandAngle = 20

        # rrt planning
        # planningStartTime = time.time()
        rrt = ma_rrt.RRT(start, planDistance, obstacleList=coneObstacleList, expandDis=expandDistance, turnAngle=expandAngle, maxIter=iterationNumber, rrtTargets = rrtConeTargets)
        nodeList, leafNodes = rrt.Planning()
        # print "rrt.Planning(): {0} ms".format((time.time() - planningStartTime) * 1000);

        # print "nodeList size: {0}".format(len(nodeList))
        # print "leafNodes size: {0}".format(len(leafNodes))

        self.publishTreeVisual(nodeList, leafNodes)

        frontConesBiggerDist = 15
        largerGroupFrontCones = self.getFrontConeObstacles(self.map, frontConesBiggerDist)

        # BestBranch
        # findBesttBranchStartTime = time.time()
        bestBranch = self.findBestBranch(leafNodes, nodeList, largerGroupFrontCones, coneObstacleSize, expandDistance, planDistance)
        # print "find best branch time: {0} ms".format((time.time() - findBesttBranchStartTime) * 1000);

        # print "best branch", bestBranch

        if bestBranch:
            filteredBestBranch = self.getFilteredBestBranch(bestBranch)
            # print "filteredBestBranch", filteredBestBranch

            if filteredBestBranch:
                # Delaunay
                # delaunayStartTime = time.time()
                delaunayEdges = self.getDelaunayEdges(frontCones)
                # print "delaunay time: {0} ms".format((time.time() - delaunayStartTime) * 1000);

                self.publishDelaunayEdgesVisual(delaunayEdges)

                # findWaypointsStartTime = time.time()

                newWaypoints = []

                if delaunayEdges:
                    # print "len(delaunayEdges):", len(delaunayEdges)
                    # print delaunayEdges

                    newWaypoints = self.getWaypointsFromEdges(filteredBestBranch, delaunayEdges)
                # else:
                #     print "newWaypoints from filteredBestBranch", newWaypoints
                #     newWaypoints = [(node.x, node.y) for node in filteredBestBranch]

                # print "find waypoints time: {0} ms".format((time.time() - findWaypointsStartTime) * 1000)

                if newWaypoints:
                    # print "newWaypoints:", waypoints

                    # mergeWaypointsStartTime = time.time()
                    self.mergeWaypoints(newWaypoints)
                    # print "merge waypoints time: {0} ms".format((time.time() - mergeWaypointsStartTime) * 1000);

                self.publishWaypoints(newWaypoints)

        # print "whole map callback: {0} ms".format((time.time() - sampleTreeStartTime)*1000);

    def mergeWaypoints(self, newWaypoints):
        # print "mergeWaypoints:", "len(saved):", len(self.savedWaypoints), "len(new):", len(newWaypoints)
        if not newWaypoints:
            return

        maxDistToSaveWaypoints = 2.0
        maxWaypointAmountToSave = 2
        waypointsDistTollerance = 0.5

        # check preliminary loopClosure
        if len(self.savedWaypoints) > 15:
            firstSavedWaypoint = self.savedWaypoints[0]

            for waypoint in reversed(newWaypoints):
                distDiff = self.dist(firstSavedWaypoint[0], firstSavedWaypoint[1], waypoint[0], waypoint[1])
                if distDiff < waypointsDistTollerance:
                    self.preliminaryLoopClosure = True
                    print "preliminaryLoopClosure = True"
                    break

        # print "savedWaypoints before:", self.savedWaypoints
        # print "newWaypoints:", newWaypoints

        newSavedPoints = []

        for i in range(len(newWaypoints)):
            waypointCandidate = newWaypoints[i]

            carWaypointDist = self.dist(self.carPosX, self.carPosY, waypointCandidate[0], waypointCandidate[1])
            # print "check candidate:", waypointCandidate, "with dist:", carWaypointDist

            if i >= maxWaypointAmountToSave or carWaypointDist > maxDistToSaveWaypoints:
                # print "condition to break:", i, i >= maxWaypointAmountToSave,  "or", (carWaypointDist > maxDistToSaveWaypoints)
                break
            else:
                for savedWaypoint in reversed(self.savedWaypoints):
                    waypointsDistDiff = self.dist(savedWaypoint[0], savedWaypoint[1], waypointCandidate[0], waypointCandidate[1])
                    if waypointsDistDiff < waypointsDistTollerance:
                        self.savedWaypoints.remove(savedWaypoint) #remove similar
                        # print "remove this point:", savedWaypoint, "with diff:", waypointsDistDiff
                        break

                if (self.preliminaryLoopClosure):
                    distDiff = self.dist(firstSavedWaypoint[0], firstSavedWaypoint[1], waypointCandidate[0], waypointCandidate[1])
                    if distDiff < waypointsDistTollerance:
                        self.loopClosure = True
                        print "loopClosure = True"
                        break

                # print "add this point:", waypointCandidate
                self.savedWaypoints.append(waypointCandidate)
                newSavedPoints.append(waypointCandidate)

        if newSavedPoints: # make self.savedWaypoints and newWaypoints having no intersection
            for point in newSavedPoints:
                newWaypoints.remove(point)

        # print "savedWaypoints after:", self.savedWaypoints
        # print "newWaypoints after:", newWaypoints

    def getWaypointsFromEdges(self, filteredBranch, delaunayEdges):
        if not delaunayEdges:
            return

        waypoints = []
        for i in range (len(filteredBranch) - 1):
            node1 = filteredBranch[i]
            node2 = filteredBranch[i+1]
            a1 = np.array([node1.x, node1.y])
            a2 = np.array([node2.x, node2.y])

            # print "node1:", node1
            # print "node2:", node2

            maxAcceptedEdgeLength = 7
            maxEdgePartsRatio = 3

            intersectedEdges = []
            for edge in delaunayEdges:
                # print "edge:", edge

                b1 = np.array([edge.x1, edge.y1])
                b2 = np.array([edge.x2, edge.y2])

                if self.getLineSegmentIntersection(a1, a2, b1, b2):
                    if edge.length() < maxAcceptedEdgeLength:
                        edge.intersection = self.getLineIntersection(a1, a2, b1, b2)

                        edgePartsRatio = edge.getPartsLengthRatio()
                        # print "edgePartsRatio:", edgePartsRatio

                        if edgePartsRatio < maxEdgePartsRatio:
                            intersectedEdges.append(edge)

            if intersectedEdges:
                # print "len(intersectedEdges):", len(intersectedEdges)
                # print "intersectedEdges:", intersectedEdges

                if len(intersectedEdges) == 1:
                    edge = intersectedEdges[0]

                    # print "edge middle:", edge.getMiddlePoint()
                    waypoints.append(edge.getMiddlePoint())
                else:
                    # print "initial:", intersectedEdges
                    intersectedEdges.sort(key=lambda edge: self.dist(node1.x, node1.y, edge.intersection[0], edge.intersection[1], shouldSqrt = False))
                    # print "sorted:", intersectedEdges

                    for edge in intersectedEdges:
                        waypoints.append(edge.getMiddlePoint())

        return waypoints

    def getDelaunayEdges(self, frontCones):
        if len(frontCones) < 4: # no sense to calculate delaunay
            return

        conePoints = np.zeros((len(frontCones), 2))

        for i in range(len(frontCones)):
            cone = frontCones[i]
            conePoints[i] = ([cone.x, cone.y])

        # print conePoints
        tri = Delaunay(conePoints)
        # print "len(tri.simplices):", len(tri.simplices)

        delaunayEdges = []
        for simp in tri.simplices:
            # print simp

            for i in range(3):
                j = i + 1
                if j == 3:
                    j = 0
                edge = Edge(conePoints[simp[i]][0], conePoints[simp[i]][1], conePoints[simp[j]][0], conePoints[simp[j]][1])

                if edge not in delaunayEdges:
                    delaunayEdges.append(edge)

        return delaunayEdges

    def dist(self, x1, y1, x2, y2, shouldSqrt = True):
        distSq = (x1 - x2) ** 2 + (y1 - y2) ** 2
        return math.sqrt(distSq) if shouldSqrt else distSq

    def publishWaypoints(self, newWaypoints = None):
        if (time.time() - self.lastPublishWaypointsTime) < self.waypointsPublishInterval:
            return

        # print "publishWaypoints(): start"
        waypointsArray = WaypointsArray()
        waypointsArray.header.frame_id = self.world_frame
        waypointsArray.header.stamp = rospy.Time.now()

        waypointsArray.preliminaryLoopClosure = self.preliminaryLoopClosure
        waypointsArray.loopClosure = self.loopClosure

        # if not self.savedWaypoints and newWaypoints:
        #     firstWaypoint = newWaypoints[0]
        #
        #     auxWaypointMaxDist = 2
        #
        #     # auxilary waypoint to start
        #     if self.dist(self.carPosX, self.carPosY, firstWaypoint[0], firstWaypoint[1]) > auxWaypointMaxDist:
        #         waypointsArray.waypoints.append(Waypoint(0, self.carPosX, self.carPosY))
        #         print "add aux point with car pos"

        for i in range(len(self.savedWaypoints)):
            waypoint = self.savedWaypoints[i]
            waypointId = len(waypointsArray.waypoints)
            w = Waypoint(waypointId, waypoint[0], waypoint[1])
            waypointsArray.waypoints.append(w)

        if newWaypoints is not None:
            for i in range(len(newWaypoints)):
                waypoint = newWaypoints[i]
                waypointId = len(waypointsArray.waypoints)
                w = Waypoint(waypointId, waypoint[0], waypoint[1])
                waypointsArray.waypoints.append(w)
                # print "added from newWaypoints:", waypointId, waypoint[0], waypoint[1]

        if self.shouldPublishWaypoints:
            # print "publish ros waypoints:", waypointsArray.waypoints
            self.waypointsPub.publish(waypointsArray)

            self.lastPublishWaypointsTime = time.time()

            self.publishWaypointsVisuals(newWaypoints)

            # print "publishWaypoints(): len(waypointsArray.waypoints):", len(waypointsArray.waypoints)
            # print "------"

    def publishWaypointsVisuals(self, newWaypoints = None):

        markerArray = MarkerArray()

        savedWaypointsMarker = Marker()
        savedWaypointsMarker.header.frame_id = self.world_frame
        savedWaypointsMarker.header.stamp = rospy.Time.now()
        savedWaypointsMarker.lifetime = rospy.Duration(1)
        savedWaypointsMarker.ns = "saved-publishWaypointsVisuals"
        savedWaypointsMarker.id = 1

        savedWaypointsMarker.type = savedWaypointsMarker.SPHERE_LIST
        savedWaypointsMarker.action = savedWaypointsMarker.ADD
        savedWaypointsMarker.pose.orientation.w = 1
        savedWaypointsMarker.scale.x = 0.15
        savedWaypointsMarker.scale.y = 0.15
        savedWaypointsMarker.scale.z = 0.15

        savedWaypointsMarker.color.a = 1.0
        savedWaypointsMarker.color.b = 1.0

        for waypoint in self.savedWaypoints:
            p = Point(waypoint[0], waypoint[1], 0.0)
            savedWaypointsMarker.points.append(p)

        markerArray.markers.append(savedWaypointsMarker)

        if newWaypoints is not None:
            newWaypointsMarker = Marker()
            newWaypointsMarker.header.frame_id = self.world_frame
            newWaypointsMarker.header.stamp = rospy.Time.now()
            newWaypointsMarker.lifetime = rospy.Duration(1)
            newWaypointsMarker.ns = "new-publishWaypointsVisuals"
            newWaypointsMarker.id = 2

            newWaypointsMarker.type = newWaypointsMarker.SPHERE_LIST
            newWaypointsMarker.action = newWaypointsMarker.ADD
            newWaypointsMarker.pose.orientation.w = 1
            newWaypointsMarker.scale.x = 0.3
            newWaypointsMarker.scale.y = 0.3
            newWaypointsMarker.scale.z = 0.3

            newWaypointsMarker.color.a = 0.65
            newWaypointsMarker.color.b = 1.0

            for waypoint in newWaypoints:
                p = Point(waypoint[0], waypoint[1], 0.0)
                newWaypointsMarker.points.append(p)

            markerArray.markers.append(newWaypointsMarker)

        self.waypointsVisualPub.publish(markerArray)

    def getLineIntersection(self, a1, a2, b1, b2):
        """
        Returns the point of intersection of the lines passing through a2,a1 and b2,b1.
        a1: [x, y] a point on the first line
        a2: [x, y] another point on the first line
        b1: [x, y] a point on the second line
        b2: [x, y] another point on the second line
        https://stackoverflow.com/questions/3252194/numpy-and-line-intersections
        """
        s = np.vstack([a1,a2,b1,b2])        # s for stacked
        h = np.hstack((s, np.ones((4, 1)))) # h for homogeneous
        l1 = np.cross(h[0], h[1])           # get first line
        l2 = np.cross(h[2], h[3])           # get second line
        x, y, z = np.cross(l1, l2)          # point of intersection
        if z == 0:                          # lines are parallel
            return (float('inf'), float('inf'))
        return (x/z, y/z)

    def getLineSegmentIntersection(self, a1, a2, b1, b2):
        # https://bryceboe.com/2006/10/23/line-segment-intersection-algorithm/
        # Return true if line segments a1a2 and b1b2 intersect
        # return ccw(A,C,D) != ccw(B,C,D) and ccw(A,B,C) != ccw(A,B,D)
        return self.ccw(a1,b1,b2) != self.ccw(a2,b1,b2) and self.ccw(a1,a2,b1) != self.ccw(a1,a2,b2)

    def ccw(self, A, B, C):
        # if three points are listed in a counterclockwise order.
        # return (C.y-A.y) * (B.x-A.x) > (B.y-A.y) * (C.x-A.x)
        return (C[1]-A[1]) * (B[0]-A[0]) > (B[1]-A[1]) * (C[0]-A[0])

    def getFilteredBestBranch(self, bestBranch):
        if not bestBranch:
            return

        everyPointDistChangeLimit = 2.0
        newPointFilter = 0.2
        maxDiscardAmountForReset = 2

        if not self.filteredBestBranch:
            self.filteredBestBranch = list(bestBranch)
        else:
            changeRate = 0
            shouldDiscard = False
            for i in range(len(bestBranch)):
                node = bestBranch[i]
                filteredNode = self.filteredBestBranch[i]

                dist = math.sqrt((node.x - filteredNode.x) ** 2 + (node.y - filteredNode.y) ** 2)
                if dist > everyPointDistChangeLimit: # changed too much, skip this branch
                    shouldDiscard = True
                    self.discardAmount += 1
                    # print "above DistChangeLimit:, shouldDiscard!,", "discAmount:", self.discardAmount

                    if self.discardAmount >= maxDiscardAmountForReset:
                        self.discardAmount = 0
                        self.filteredBestBranch = list(bestBranch)
                        # print "broke maxDiscardAmountForReset:, Reset!"
                    break

                changeRate += (everyPointDistChangeLimit - dist)
            # print "branch changeRate: {0}".format(changeRate);

            if not shouldDiscard:
            #     return
            # else:
                for i in range(len(bestBranch)):
                    self.filteredBestBranch[i].x = self.filteredBestBranch[i].x * (1 - newPointFilter) + newPointFilter * bestBranch[i].x
                    self.filteredBestBranch[i].y = self.filteredBestBranch[i].y * (1 - newPointFilter) + newPointFilter * bestBranch[i].y

                self.discardAmount = 0
                # print "reset discardAmount, ", "discAmount:", self.discardAmount

        self.publishFilteredBranchVisual()
        return list(self.filteredBestBranch) # return copy

    def publishDelaunayEdgesVisual(self, edges):
        if not edges:
            return

        marker = Marker()
        marker.header.frame_id = self.world_frame
        marker.header.stamp = rospy.Time.now()
        marker.lifetime = rospy.Duration(1)
        marker.ns = "publishDelaunayLinesVisual"

        marker.type = marker.LINE_LIST
        marker.action = marker.ADD
        marker.scale.x = 0.05

        marker.pose.orientation.w = 1

        marker.color.a = 0.5
        marker.color.r = 1.0
        marker.color.b = 1.0

        for edge in edges:
            # print edge

            p1 = Point(edge.x1, edge.y1, 0)
            p2 = Point(edge.x2, edge.y2, 0)

            marker.points.append(p1)
            marker.points.append(p2)

        self.delaunayLinesVisualPub.publish(marker)

    def findBestBranch(self, leafNodes, nodeList, largerGroupFrontCones, coneObstacleSize, expandDistance, planDistance):
        if not leafNodes:
            return

        coneDistLimit = 4.0
        coneDistanceLimitSq = coneDistLimit * coneDistLimit;

        bothSidesImproveFactor = 3
        minAcceptableBranchRating = 80 # fits good fsg18

        leafRatings = []
        for leaf in leafNodes:
            branchRating = 0
            node = leaf
            # print " ===== calculating leaf node {0} ====== ".format(leaf)
            while node.parent is not None:
                nodeRating = 0
                # print "---- check node {0}".format(node)

                leftCones = []
                rightCones = []

                for cone in largerGroupFrontCones:
                    coneDistSq = ((cone.x - node.x) ** 2 + (cone.y - node.y) ** 2)

                    if coneDistSq < coneDistanceLimitSq:
                        actualDist = math.sqrt(coneDistSq)

                        if actualDist < coneObstacleSize:
                            # node can be really close to a cone, cause we have new cones in this comparison, so skip these ones
                            continue

                        nodeRating += (coneDistLimit - actualDist)
                        # print "found close cone({1},{2}), rating: {0}".format(nodeRating, cone.x, cone.y)

                        if self.isLeftCone(node, nodeList[node.parent], cone):
                            leftCones.append(cone)
                        else:
                            rightCones.append(cone)

                if ((len(leftCones) == 0 and len(rightCones)) > 0 or (len(leftCones) > 0 and len(rightCones) == 0)):
                    # print "cones are only from one side, penalize rating"
                    nodeRating /= bothSidesImproveFactor

                if (len(leftCones) > 0 and len(rightCones) > 0):
                    # print "cones are from both sides, improve rating"
                    nodeRating *= bothSidesImproveFactor

                # print "node.cost: {0}, node.rating: {1}".format(node.cost, nodeRating)

                # make conversion: (expandDistance to planDistance) -> (1 to 2)
                nodeFactor = (node.cost - expandDistance)/(planDistance - expandDistance) + 1
                # print "nodeFactor: {0}".format(nodeFactor)

                branchRating += nodeRating * nodeFactor
                # branchRating += nodeRating
                # print "current branch rating: {0}".format(branchRating)
                node = nodeList[node.parent]

            leafRatings.append(branchRating)
            # print "leaf node {0}, rating: {1}".format(leaf, branchRating)

        # print leafRatings
        maxRating = max(leafRatings)
        maxRatingInd = leafRatings.index(maxRating)

        node = leafNodes[maxRatingInd]
        # print "!!maxRating leaf node {0}, rating: {1}".format(node, maxRating)

        if maxRating < minAcceptableBranchRating:
            return

        self.publishBestBranchVisual(nodeList, node)

        reverseBranch = []
        reverseBranch.append(node)
        while node.parent is not None:
            node = nodeList[node.parent]
            reverseBranch.append(node)

        directBranch = []
        for n in reversed(reverseBranch):
            directBranch.append(n)
            # print n

        return directBranch

    def isLeftCone(self, node, parentNode, cone):
        # //((b.X - a.X)*(cone.Y - a.Y) - (b.Y - a.Y)*(cone.X - a.X)) > 0;
        return ((node.x - parentNode.x) * (cone.y - parentNode.y) - (node.y - parentNode.y) * (cone.x - parentNode.x)) > 0;

    def publishBestBranchVisual(self, nodeList, leafNode):
        marker = Marker()
        marker.header.frame_id = self.world_frame
        marker.header.stamp = rospy.Time.now()
        marker.lifetime = rospy.Duration(0.2)
        marker.ns = "publishBestBranchVisual"

        marker.type = marker.LINE_LIST
        marker.action = marker.ADD
        marker.scale.x = 0.07

        marker.pose.orientation.w = 1

        marker.color.a = 0.7
        marker.color.r = 1.0

        node = leafNode

        parentNodeInd = node.parent
        while parentNodeInd is not None:
            parentNode = nodeList[parentNodeInd]
            p = Point(node.x, node.y, 0)
            marker.points.append(p)

            p = Point(parentNode.x, parentNode.y, 0)
            marker.points.append(p)

            parentNodeInd = node.parent
            node = parentNode

        self.bestBranchVisualPub.publish(marker)

    def publishFilteredBranchVisual(self):

        if not self.filteredBestBranch:
            return

        marker = Marker()
        marker.header.frame_id = self.world_frame
        marker.header.stamp = rospy.Time.now()
        marker.lifetime = rospy.Duration(0.2)
        marker.ns = "publisshFilteredBranchVisual"

        marker.type = marker.LINE_LIST
        marker.action = marker.ADD
        marker.scale.x = 0.07

        marker.pose.orientation.w = 1

        marker.color.a = 0.7
        marker.color.b = 1.0

        for i in range(len(self.filteredBestBranch)):
            node = self.filteredBestBranch[i]
            p = Point(node.x, node.y, 0)
            if i != 0:
                marker.points.append(p)

            if i != len(self.filteredBestBranch) - 1:
                marker.points.append(p)

        self.filteredBranchVisualPub.publish(marker)

    def publishTreeVisual(self, nodeList, leafNodes):

        if not nodeList and not leafNodes:
            return

        markerArray = MarkerArray()

        # tree lines marker
        treeMarker = Marker()
        treeMarker.header.frame_id = self.world_frame
        treeMarker.header.stamp = rospy.Time.now()
        treeMarker.ns = "rrt"

        treeMarker.type = treeMarker.LINE_LIST
        treeMarker.action = treeMarker.ADD
        treeMarker.scale.x = 0.03

        treeMarker.pose.orientation.w = 1

        treeMarker.color.a = 0.7
        treeMarker.color.g = 0.7

        treeMarker.lifetime = rospy.Duration(0.2)

        for node in nodeList:
            if node.parent is not None:
                p = Point(node.x, node.y, 0)
                treeMarker.points.append(p)

                p = Point(nodeList[node.parent].x, nodeList[node.parent].y, 0)
                treeMarker.points.append(p)

        markerArray.markers.append(treeMarker)

        # leaves nodes marker
        leavesMarker = Marker()
        leavesMarker.header.frame_id = self.world_frame
        leavesMarker.header.stamp = rospy.Time.now()
        leavesMarker.lifetime = rospy.Duration(0.2)
        leavesMarker.ns = "rrt-leaves"

        leavesMarker.type = leavesMarker.SPHERE_LIST
        leavesMarker.action = leavesMarker.ADD
        leavesMarker.pose.orientation.w = 1
        leavesMarker.scale.x = 0.15
        leavesMarker.scale.y = 0.15
        leavesMarker.scale.z = 0.15

        leavesMarker.color.a = 0.5
        leavesMarker.color.b = 0.1

        for node in leafNodes:
            p = Point(node.x, node.y, 0)
            leavesMarker.points.append(p)

        markerArray.markers.append(leavesMarker)

        # publis marker array
        self.treeVisualPub.publish(markerArray)

    def getFrontConeObstacles(self, map, frontDist):
        if not map:
            return []

        headingVector = self.getHeadingVector()
        # print("headingVector:", headingVector)

        headingVectorOrt = [-headingVector[1], headingVector[0]]
        # print("headingVectorOrt:", headingVectorOrt)

        behindDist = 0.5
        carPosBehindPoint = [self.carPosX - behindDist * headingVector[0], self.carPosY - behindDist * headingVector[1]]

        # print "carPos:", [self.carPosX, self.carPosY]
        # print "carPosBehindPoint:", carPosBehindPoint

        frontDistSq = frontDist ** 2

        frontConeList = []
        for cone in map:
            if (headingVectorOrt[0] * (cone.y - carPosBehindPoint[1]) - headingVectorOrt[1] * (cone.x - carPosBehindPoint[0])) < 0:
                if ((cone.x - self.carPosX) ** 2 + (cone.y - self.carPosY) ** 2) < frontDistSq:
                    frontConeList.append(cone)
        return frontConeList

    def getHeadingVector(self):
        headingVector = [1.0, 0]
        carRotMat = np.array([[math.cos(self.carPosYaw), -math.sin(self.carPosYaw)], [math.sin(self.carPosYaw), math.cos(self.carPosYaw)]])
        headingVector = np.dot(carRotMat, headingVector)
        return headingVector

    def getConesInRadius(self, map, x, y, radius):
        coneList = []
        radiusSq = radius * radius
        for cone in map:
            if ((cone.x - x) ** 2 + (cone.y - y) ** 2) < radiusSq:
                coneList.append(cone)
        return coneList

class Edge():
    def __init__(self, x1, y1, x2, y2):
        self.x1 = x1
        self.y1 = y1
        self.x2 = x2
        self.y2 = y2
        self.intersection = None

    def getMiddlePoint(self):
        return (self.x1 + self.x2) / 2, (self.y1 + self.y2) / 2

    def length(self):
        return math.sqrt((self.x1 - self.x2) ** 2 + (self.y1 - self.y2) ** 2)

    def getPartsLengthRatio(self):
        import math

        part1Length = math.sqrt((self.x1 - self.intersection[0]) ** 2 + (self.y1 - self.intersection[1]) ** 2)
        part2Length = math.sqrt((self.intersection[0] - self.x2) ** 2 + (self.intersection[1] - self.y2) ** 2)

        return max(part1Length, part2Length) / min(part1Length, part2Length)

    def __eq__(self, other):
        return (self.x1 == other.x1 and self.y1 == other.y1 and self.x2 == other.x2 and self.y2 == other.y2
             or self.x1 == other.x2 and self.y1 == other.y2 and self.x2 == other.x1 and self.y2 == other.y1)

    def __str__(self):
        return "(" + str(round(self.x1, 2)) + "," + str(round(self.y1,2)) + "),(" + str(round(self.x2, 2)) + "," + str(round(self.y2,2)) + ")"

    def __repr__(self):
        return str(self)

if __name__ == '__main__':

    a1 = np.array([0, 0])
    a2 = np.array([5, 0])
    b1 = np.array([0, 5])
    b2 = np.array([5, 0])

    maNode = MaRRTPathPlanNode()

    if maNode.getLineSegmentIntersection(a1, a2, b1, b2):
        print "intersected"
    else:
        print "not intersected"
