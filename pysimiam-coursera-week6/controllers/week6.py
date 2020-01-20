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

class FollowWall(PIDController):
    """Follow walls is a controller that keeps a certain distance
    to the wall and drives alongside it in clockwise or counter-clockwise
    fashion."""
    def __init__(self, params):
        '''Initialize internal variables'''
        PIDController.__init__(self,params)

        # This variable should contain a list of vectors
        # calculated from the relevant sensor readings.
        # It is used by the supervisor to draw & debug
        # the controller's behaviour
        self.vectors = []
        
    def restart(self):
        """Reset internal state"""
        PIDController.restart(self)

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


        # print(self.direction)

    def get_heading(self, state):
        """Get the direction along the wall as a vector."""
        
        # Week 6 Assignment:
        self.sensor_distances = state.sensor_distances
        # Calculate vectors for the sensors
        self.vectors = []
        ir_distances = state.sensor_distances;
        for i, pose in enumerate(self.sensor_poses):
            R = pose.get_transformation()
            self.vectors.append(numpy.dot(R, numpy.array([ir_distances[i], 0, 1])))
        if self.direction== 'left':
            sorting_list = self.sensor_distances[-1:-4:-1]
            sorting_list = numpy.argsort(sorting_list)
            if sorting_list[0] > sorting_list[1]:
                point1 = self.vectors[-sorting_list[1]-1]
                point2 = self.vectors[-sorting_list[0]-1]
            else:
                point1 = self.vectors[-sorting_list[0]-1]
                point2 = self.vectors[-sorting_list[1]-1]
        else:
            sorting_list = self.sensor_distances[0:3]
            sorting_list = numpy.argsort(sorting_list)

            if sorting_list[0] > sorting_list[1]:
                point1 = self.vectors[sorting_list[1]]
                point2 = self.vectors[sorting_list[0]]
            else:
                point1 = self.vectors[sorting_list[0]]
                point2 = self.vectors[sorting_list[1]]
        self.along_wall_vector = point2 - point1
        # vector perpendicular to wall
        dist_along_wall_vector = numpy.linalg.norm(numpy.array([self.along_wall_vector[0], self.along_wall_vector[1]]))
        normalized_along_wall_vector = self.along_wall_vector[:2]/dist_along_wall_vector
        vector_sub = point1[:2]# - [x_r, y_r]
        self.to_wall_vector = vector_sub - numpy.dot(vector_sub, normalized_along_wall_vector)*normalized_along_wall_vector

        # Calculate and return the heading vector:
        dist_to_wall_vector = numpy.linalg.norm(numpy.array([self.to_wall_vector[0], self.to_wall_vector[1]]))
        v_to_wall_prop = self.to_wall_vector - self.distance*(self.to_wall_vector/dist_to_wall_vector)
        return self.distance*normalized_along_wall_vector + v_to_wall_prop

        # End Week 6 Assignment
               