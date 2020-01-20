#
# (c) PySimiam Team 2013
# 
# Contact person: Tim Fuchs <typograph@elec.ru>
#
# This class was implemented for the weekly programming excercises
# of the 'Control of Mobile Robots' course by Magnus Egerstedt.
#
from controllers.pid_controller import PIDController
import math
import numpy
from pose import Pose

class FollowWall(PIDController):
    """Follow walls is a controller that keeps a certain distance
    to the wall and drives alongside it in clockwise or counter-clockwise
    fashion."""
    def __init__(self, params):
        '''Initialize internal variables'''
        PIDController.__init__(self,params)

    def set_parameters(self, params):
        """Set PID values, sensor poses, direction and distance.
        
        The params structure is expected to have sensor poses in the robot's
        reference frame as ``params.sensor_poses``, the direction of wall
        following (either 'right' for clockwise or 'left' for anticlockwise)
        as ``params.direction`` and the desired distance to the wall 
        to maintain as ``params.distance``.
        """
        PIDController.set_parameters(self,params)

        self.sensor_poses = params.sensor_poses
        self.direction = params.direction
        self.distance = params.distance

    def mycode(self, state):

        self.sensor_distances = state.sensor_distances
        # Calculate vectors for the sensors
        self.vectors = []
        ir_distances = state.sensor_distances;
        for i, pose in enumerate(self.sensor_poses):
            R = pose.get_transformation()
            self.vectors.append(numpy.dot(R, numpy.array([ir_distances[i], 0, 1])))
        #here we find out indexes of sensors that sense wall
        if state.direction == 'right': #sensors 2-4
            sorting_list = self.sensor_distances[2:]
            if sorting_list[0] < sorting_list[2]:
                i = 2
                j = 3
            else:
                i = 3
                j = 4
        else: #sensors 0-2
            sorting_list = self.sensor_distances[:3]
            if sorting_list[2] < sorting_list[0]:
                i = 2
                j = 1
            else:
                i = 1
                j = 0
        point1 = Pose(self.vectors[j])
        point2 = Pose(self.vectors[i])
        self.along_wall_vector = numpy.array([point2.x-point1.x, point2.y-point1.y])
        # vector perpendicular to wall
        dist_along_wall_vector = numpy.linalg.norm(self.along_wall_vector)
        normalized_along_wall_vector = self.along_wall_vector / dist_along_wall_vector
        vector_sub = numpy.array([point1.x, point1.y])
        self.to_wall_vector = vector_sub - numpy.dot(vector_sub,
                                                     normalized_along_wall_vector) * normalized_along_wall_vector


        # Calculate and return the heading vector:
        dist_to_wall_vector = numpy.linalg.norm(self.to_wall_vector)

        v_to_wall_prop = self.to_wall_vector - self.distance * (self.to_wall_vector / dist_to_wall_vector)
        # print('my vectors')
        self.to_wall_vector = v_to_wall_prop
        # print(self.to_wall_vector)
        # print(self.along_wall_vector)
        # print(normalized_along_wall_vector)
        # print(v_to_wall_prop)
        # print('my result vector')
        # print(normalized_along_wall_vector + v_to_wall_prop)
        return self.distance * normalized_along_wall_vector + (1-self.distance)*v_to_wall_prop

    def theircode(self, state):
        # Calculate vectors for the sensors

        if state.direction == 'left': # 0-2
            d, i = min( zip(state.sensor_distances[:3],[0,1,2]) )
            if i == 0 or (i == 1 and state.sensor_distances[0] <= state.sensor_distances[2]):
                i, j, k = 1, 0, 2

            else:
                i, j, k = 2, 1, 0

        else : # 2-4
            d, i = min( zip(state.sensor_distances[2:],[2,3,4]) )
            if i == 4 or (i == 3 and state.sensor_distances[4] <= state.sensor_distances[2]):
                i, j, k = 3, 4, 2
            else:
                i, j, k = 2, 3, 4

        p_front = Pose(state.sensor_distances[i]) >> self.sensor_poses[i]
        p_back = Pose(state.sensor_distances[j]) >> self.sensor_poses[j]

        self.vectors = [(p_front.x,p_front.y,1), (p_back.x, p_back.y, 1)]

        # Calculate the two vectors:
        ds = ((p_front.x-p_back.x)**2 + (p_front.y-p_back.y)**2)
        ms = (p_front.x*p_back.y - p_front.y*p_back.x)
        self.to_wall_vector = numpy.array([(p_back.y-p_front.y)*ms/ds,(p_front.x-p_back.x)*ms/ds,1])
        self.along_wall_vector = numpy.array([p_front.x-p_back.x, p_front.y-p_back.y, 1])

        # Calculate and return the heading vector:
        offset = abs(ms/math.sqrt(ds)) - state.distance
        print('their vectors')
        print(self.to_wall_vector)
        print(self.along_wall_vector)
        print('their result vectors')
        print(0.3*self.along_wall_vector + 2 * offset * self.to_wall_vector)
        print(0.3 * self.along_wall_vector + 1 * offset * self.to_wall_vector)
        if offset > 0:
            return 0.3*self.along_wall_vector + 3 * offset * self.to_wall_vector
        else:
            return 0.3*self.along_wall_vector + 2 * offset * self.to_wall_vector

    def get_heading(self, state):
        """Get the direction along the wall as a vector."""
        # self.mycode(state)
        # return self.theircode(state)
        return self.mycode(state)


