from typing import Tuple

from pygame import Vector2
import pygame.draw

from ...bot import Bot
from ...linear_math import Transform

DEBUG_MODE = False

class CommandValue():
    def __init__(self, value : float):
        self.value = value

    def __str__(self):
        return str(self.value)

    def __repr__(self):
        return "CommandValue(" + str(self.value) + ")"
    
    def __float__(self):
        return self.value

    def __gt__(self, other):
        (gt,*diff) = (other > 0, self.value > 0)
        if diff[0] and gt:
            return False
        if diff[0] and not gt:
            return True
        if not diff[0] and gt:
            return False
        else:
            return True
   
    def __mul__(self, other):
        return self.value * other
    
class MainBot(Bot):

    def __init__(self, track, training_wheels=True, name="MainBot", contributor="Anonymous"):
        super().__init__(track)

        self.fps = 60.0
        self.dt = 1.0/self.fps
        self.max_throttle = 100
        self.max_steering_speed = 3
        self.slipping_acceleration = 200
        self.slipping_ratio = 0.6
        
        self.latest_command = None
        
        self.simple_state = 0
        
        self.smoother_velocity_target = 5
        self.first_step = True
        self.new_waypoint = False
        self.current_waypoint = 0
        self.simple_max_speed = 105.0
        self.simple_steering_speed = 1.0
        self.training_wheels = training_wheels
        self.prevous_distance = 0.0

        self.int_name = name
        self.int_contributor = contributor


    @property
    def name(self):
        return self.int_name

    @property
    def contributor(self):
        return "Anonymous"

    def compute_commands(self, next_waypoint: int, position: Transform, velocity: Vector2) -> Tuple:
        self.simple_max_speed = min(200.0, self.simple_max_speed + 0.1)
        if DEBUG_MODE:
            print(self.simple_max_speed)

        self.calc_and_store(next_waypoint, position, velocity)

        if self.training_wheels:
            self.latest_command = self.move_smooth()
        else:
            self.latest_command = self.move_simple()

        if DEBUG_MODE:
            print(f"Latest command: {self.latest_command}")
        return self.latest_command
    
    def move_simple(self):
        if DEBUG_MODE:
            print("Moving simple")
        if self.simple_state == 0:
            if DEBUG_MODE:
                print("Rotating towards target")
            self.simple_state = 1
            return CommandValue(0), CommandValue(self.target.as_polar()[1]/180.0 * 3.14159265359*self.fps/self.max_steering_speed)
        if self.simple_state == 1:
            if DEBUG_MODE:
                print("Moving forward")
            self.simple_state = 2
            return CommandValue(self.target.length()*self.fps*self.fps/self.max_throttle), CommandValue(0)
        if self.simple_state == 2:
            if DEBUG_MODE:
                print("Slowing down")
            self.simple_state = 0
            return CommandValue(-self.current_velocity.length()*self.fps/self.max_throttle), CommandValue(0)

    def move_smooth(self):
        if DEBUG_MODE:
            print("Moving simple")
        if self.simple_state == 0:
            if DEBUG_MODE:
                print("Rotating towards target")
            target_angle = self.target.as_polar()[1]/180.0 * 3.14159265359
            target_steering_speed = min(self.simple_steering_speed,target_angle*self.fps/self.max_steering_speed)
            if DEBUG_MODE:
                print("Target angle: ", target_angle)
            if abs(target_angle) < 0.01:
                self.simple_state = 1
                target_steering_speed = target_angle*self.fps/self.max_steering_speed
            return CommandValue(0), CommandValue(target_steering_speed)
        if self.simple_state == 1:
            if DEBUG_MODE:
                print("Moving forward")
            self.prevous_distance = self.target.length()
            throttle = min(self.simple_max_speed, self.target.length()*self.fps)*self.fps/self.max_throttle
            self.simple_state = 2
            return CommandValue(throttle), CommandValue(0)
        if self.simple_state == 2:
            if self.new_waypoint or self.prevous_distance < self.target.length():
                self.simple_state = 3
            self.prevous_distance = self.target.length()
            return CommandValue(0), CommandValue(0)
        if self.simple_state == 3:
            if DEBUG_MODE:
                print("Slowing down")
            self.simple_state = 0
            return CommandValue(-self.current_velocity.length()*self.fps/self.max_throttle*0.55), CommandValue(0)
           
    def calc_and_store(self, next_waypoint, position, velocity):
        if self.current_waypoint != next_waypoint:
            self.new_waypoint = True
            self.current_waypoint = next_waypoint
        else:
            self.new_waypoint = False
        self.current_position = position
        
        target = position.inverse() * self.track.lines[next_waypoint]
        self.target = target

        current_velocity = position.M.transpose() * velocity
        self.current_velocity = current_velocity
        
    def draw(self, map_scaled, zoom):
        if DEBUG_MODE:
            pygame.draw.line(
                map_scaled,
                pygame.Color(0, 0, 0, 50),
                self.current_position.p * zoom,
                (self.current_position * self.target) * zoom,
                2,
            )
            pygame.draw.line(
                map_scaled,
                pygame.Color(0, 0, 255, 50),
                self.current_position.p * zoom,
                self.current_position * (self.current_velocity * self.dt) * zoom,
                2,
            )
            # print(f"Latest command: {self.latest_command}")
    
class PedaltotheMetal(MainBot):

    def __init__(self, track):
        super().__init__(track, False, "PedaltotheMetal", contributor="Rinus")

class SmoothSailing(MainBot):

    def __init__(self, track):
        super().__init__(track, True, "SmoothSailing", contributor="Rinus")