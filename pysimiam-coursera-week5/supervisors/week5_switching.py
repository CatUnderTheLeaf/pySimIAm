#
# (c) PySimiam Team 2013
# 
# Contact person: Tim Fuchs <typograph@elec.ru>
#
# This class was implemented for the weekly programming excercises
# of the 'Control of Mobile Robots' course by Magnus Egerstedt.
#
from supervisors.quickbot import QuickBotSupervisor
from supervisor import Supervisor
from math import sqrt, sin, cos, atan2

class QBSwitchingSupervisor(QuickBotSupervisor):
    """QBSwitching supervisor switches between go-to-goal and avoid-obstacles controllers to make the robot reach the goal smoothly and without colliding wth walls."""
    def __init__(self, robot_pose, robot_info):
        """Create necessary controllers"""
        QuickBotSupervisor.__init__(self, robot_pose, robot_info)

        # Fill in poses for the controller
        self.parameters.sensor_poses = robot_info.ir_sensors.poses[:]

        # Create the controllers
        self.avoidobstacles = self.create_controller('AvoidObstacles', self.parameters)
        self.gtg = self.create_controller('GoToGoal', self.parameters)
        self.hold = self.create_controller('Hold', None)
        self.blending = self.create_controller('week5.Blending', self.parameters)

        # Create some state transitions
        self.add_controller(self.hold)
        # Part1 only switch
        # self.add_controller(self.gtg, \
        #                     (self.at_goal, self.hold), \
        #                     (self.at_obstacle, self.avoidobstacles))
        # self.add_controller(self.avoidobstacles, \
        #                     (self.at_goal, self.hold), \
        #                     (self.obstacle_cleared, self.gtg))
        
        # Week 5 Assigment code should go here
        # Part2 with blending
        self.add_controller(self.gtg, \
                            (self.at_goal, self.hold), \
                            (self.at_obstacle, self.blending))
        self.add_controller(self.blending, \
                            (self.at_goal, self.hold), \
                            (self.obstacle_cleared, self.gtg), \
                            (self.unsafe, self.avoidobstacles))
        self.add_controller(self.avoidobstacles, \
                            (self.at_goal, self.hold), \
                            (self.safe, self.blending))
        # End Week 5 Assignment
        
        # Start in 'go-to-goal' state
        self.current = self.gtg

    def set_parameters(self,params):
        """Set parameters for itself and the controllers"""
        QuickBotSupervisor.set_parameters(self,params)
        self.gtg.set_parameters(self.parameters)
        self.avoidobstacles.set_parameters(self.parameters)
        self.blending.set_parameters(self.parameters)

    def at_goal(self):
        """Check if the distance to goal is small"""
        # Week 5 Assigment code should go here
        
        # End Week 5 Assignment
        return self.distance_from_goal < self.robot.wheels.base_length/2
        
    def at_obstacle(self):
        """Check if the distance to obstacle is small"""
        # Week 5 Assigment code should go here
        return self.distmin < self.robot.ir_sensors.rmax/2
        
    def obstacle_cleared(self):
        """Check if the distance to obstacle is large"""
        # Week 5 Assigment code should go here
        
        # End Week 5 Assignment
        return self.distmin > self.robot.ir_sensors.rmax/1.1

    def safe(self):
        """Check if the distance to obstacle is small"""
        # Week 5 Assigment code should go here
        return self.distmin > self.robot.ir_sensors.rmax/1.2

    def unsafe(self):
        """Check if the distance to obstacle is small"""
        # Week 5 Assigment code should go here
        return self.distmin < self.robot.ir_sensors.rmax/1.2

    def process_state_info(self, state):
        """Update state parameters for the controllers and self"""

        QuickBotSupervisor.process_state_info(self,state)

        # The pose for controllers
        self.parameters.pose = self.pose_est
        # Sensor readings in real units
        self.parameters.sensor_distances = self.get_ir_distances()
        
        # Week 5 Assigment code can go here
        self.distance_from_goal = sqrt(
            (self.pose_est.x - self.parameters.goal.x) ** 2 + (self.pose_est.y - self.parameters.goal.y) ** 2)
        self.distmin = min(self.parameters.sensor_distances)
        # End Week 5 Assignment
            
    def draw_foreground(self, renderer):
        """Draw controller info"""
        QuickBotSupervisor.draw_foreground(self,renderer)

        renderer.set_pose(self.pose_est)
        arrow_length = self.robot_size*5

        # Ensure the headings are calculated
        away_angle = self.avoidobstacles.get_heading_angle(self.parameters)
        goal_angle = self.gtg.get_heading_angle(self.parameters)
        
        # Draw arrow to goal
        if self.current == self.gtg:
            renderer.set_pen(0x00FF00,0.01)
        else:
            renderer.set_pen(0xA000FF00)
        renderer.draw_arrow(0,0,
            arrow_length*cos(goal_angle),
            arrow_length*sin(goal_angle))

        # Draw arrow away from obstacles
        if self.current == self.avoidobstacles:
            renderer.set_pen(0xFF0000,0.01)
        else:
            renderer.set_pen(0xA0FF0000)
        renderer.draw_arrow(0,0,
            arrow_length*cos(away_angle),
            arrow_length*sin(away_angle))

        if "blending" in self.__dict__:
            blend_angle = self.blending.get_heading_angle(self.parameters)
            # Draw the blending
            if self.current == self.blending:
                renderer.set_pen(0xFF, 0.01)
            else:
                renderer.set_pen(0xA00000FF)
            renderer.draw_arrow(0,0,
                arrow_length*cos(blend_angle),
                arrow_length*sin(blend_angle))
            