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

class AvoidObstacles(PIDController):
    """Avoid obstacles is an example controller that checks the sensors
       for any readings, constructs 'obstacle' vectors and directs the robot
       in the direction of their weightd sum."""
    def __init__(self, params):
        '''Initialize internal variables'''
        PIDController.__init__(self,params)
        
        # This variable should contain a list of vectors
        # calculated from sensor readings. It is used by
        # the supervisor to draw & debug the controller's
        # behaviour
        self.vectors = []

    def set_parameters(self, params):
        """Set PID values and sensor poses.
        
        The params structure is expected to have sensor poses in the robot's
        reference frame as ``params.sensor_poses``.
        """
        PIDController.set_parameters(self,params)

        self.sensor_poses = params.sensor_poses

        
        # Week 4 assigment
        # Set the weigths here
        self.weights = [1, 1.5, 1, 1.5, 1]


    def get_heading(self, state):
        """Get the direction away from the obstacles as a vector."""
        
        # Week 4 Assignment:
        # Calculate vectors:
        self.vectors = []
        ir_distances = state.sensor_distances;
        for i, pose in enumerate(self.sensor_poses):
            R = pose.get_transformation()
            self.vectors.append(numpy.dot(R, numpy.array([ir_distances[i], 0, 1])))
        if numpy.amin(ir_distances) < numpy.amax(ir_distances):
            state.velocity.v = 0.2
        else:
            state.velocity.v = 0.9
        # multiply each vector by corresponding weight
        heading = numpy.array(self.weights)[:, numpy.newaxis]*numpy.array(self.vectors)
        heading = numpy.sum(heading, axis=0)
        theta_ao = numpy.arctan2(heading[1], heading[0])
        heading_angle = theta_ao - state.pose.get_list()[2]
        heading_angle = numpy.arctan2(numpy.sin(heading_angle), numpy.cos(heading_angle))
        heading[2] = heading_angle
        # print(theta_ao)
        # print(heading)
        # Calculate weighted sum:
        # heading = [1, 0, 1]
     
        # End Week 4 Assignment

        return heading