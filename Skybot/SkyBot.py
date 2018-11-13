import math
import time

class Agent:
    def __init__(self, name, team, index):
        self.index = index

        # Controller inputs
        self.throttle = 0
        self.steer = 0
        self.pitch = 0
        self.yaw = 0
        self.roll = 0
        self.boost = False
        self.jump = False
        self.powerslide = False

        # Game values
        self.bot_loc_x = None
        self.bot_loc_y = None
        self.bot_loc_z = None
        self.bot_speed_x = None
        self.bot_speed_y = None
        self.bot_speed_z = None
        self.bot_rot_yaw = None
        self.bot_rot_roll = None
        self.bot_rot_pitch = None
        self.bot_jumped = False
        self.bot_doublejumped = False
        self.bot_ground = False
        self.bot_sonic = False
        self.bot_dodge = False
        self.bot_boost = None
        self.ball_loc_x = None
        self.ball_loc_y = None
        self.ball_loc_z = None
        self.ball_speed_x = None
        self.ball_speed_y = None
        self.ball_speed_z = None
        self.ball_acc_x = None

        #game values converted
        self.bot_yaw = None
        self.bot_pitch = None
        self.bot_roll = None
        self.angle_front_to_target = None
        self.angle_car_ball = None

        #custom values
        self.angle_car_ball = None
        self.distance_car_ball = None
        self.bot_speed_linear = None

        self.after_dodge = False
        self.next_dodge_time = None

    def distance(self, x1, y1, z1, x2, y2, z2):
        distance=math.sqrt((x2 - x1)**2 + (y2 - y1)**2 + (z2 - z1)**2)
        return distance

    def dodge(self):
        if self.bot_doublejumped:
            self.jump=False
        elif not self.bot_jumped:
            self.jump=True
            self.pitch=-1
            self.next_dodge_time = time.time() + 0.15
        elif  time.time() > self.next_dodge_time:
            self.jump=True
            self.pitch=-1
            self.after_dodge=True

    def aim(self, target_x, target_y):
        self.throttle=1
        angle_between_bot_and_target = math.degrees(math.atan2(target_y - self.bot_loc.Y, target_x - self.bot_loc.X))
        self.angle_front_to_target = angle_between_bot_and_target - self.bot_rot_yaw
        # Correct the values
        if self.angle_front_to_target < -180:
            self.angle_front_to_target += 360
        if self.angle_front_to_target > 180:
            self.angle_front_to_target -= 360

        if self.angle_front_to_target < -10:
            # If the target is more than 10 degrees right from the centre, steer left
            self.steer = -1
        elif self.angle_front_to_target > 10:
            # If the target is more than 10 degrees left from the centre, steer right
            self.steer = 1
        else:
            # If the target is less than 10 degrees from the centre, steer straight
            self.steer = self.angle_front_to_target/10

        if -3<self.angle_front_to_target<3:
            self.boost=True
        else: self.boost=False
        if self.angle_front_to_target >100 or self.angle_front_to_target <-100:
            self.powerslide = 1
        else: self.powerslide = 0

    def get_output_vector(self, values):
        self.boost = False
        self.jump=False

        # Update game data variables
        self.bot_loc= values.gamecars[self.index].Location
        self.bot_rot= values.gamecars[self.index].Rotation
        self.ball_loc= values.gameball.Location

        #get values
        self.bot_loc_x = values.gamecars[self.index].Location.X
        self.bot_loc_y = values.gamecars[self.index].Location.Y
        self.bot_loc_z = values.gamecars[self.index].Location.Z
        self.bot_speed_x = values.gamecars[self.index].Velocity.X
        self.bot_speed_y = values.gamecars[self.index].Velocity.Y
        self.bot_speed_z = values.gamecars[self.index].Velocity.Z
        self.bot_jumped = values.gamecars[self.index].bJumped
        self.bot_doublejumped = values.gamecars[self.index].bDoubleJumped
        self.bot_sonic = values.gamecars[self.index].bSuperSonic
        self.bot_ground = values.gamecars[self.index].bOnGround
        self.bot_boost = values.gamecars[self.index].Boost
        self.ball_loc_x = values.gameball.Location.X
        self.ball_loc_y = values.gameball.Location.Y
        self.ball_loc_z = values.gameball.Location.Z
        self.ball_speed_x = values.gameball.Velocity.X
        self.ball_speed_y = values.gameball.Velocity.Y
        self.ball_speed_z = values.gameball.Velocity.Z
        self.ball_acc_x = values.gameball.Acceleration.Z

        # Get car's yaw, pitch and roll and convert from Unreal Rotator units to degrees
        self.bot_rot_yaw = abs(self.bot_rot.Yaw) % 65536 / 65536 * 360
        if self.bot_rot.Yaw < 0:
            self.bot_rot_yaw *= -1
        self.bot_rot_pitch = abs(self.bot_rot.Pitch) % 65536 / 65536 * 360
        if self.bot_rot.Pitch < 0:
            self.bot_rot_pitch *= -1
        self.bot_rot_roll = abs(self.bot_rot.Roll) % 65536 / 65536 * 360
        if self.bot_rot.Roll < 0:
            self.bot_rot_roll *= -1

        #get values
        self.angle_car_ball = math.degrees(math.atan2(self.ball_loc_y - self.bot_loc.Y, self.ball_loc_x - self.bot_loc.X)) - self.bot_rot_yaw
        self.distance_car_ball = self.distance(self.bot_loc_x, self.bot_loc_y, self.bot_loc_z, self.ball_loc_x, self.ball_loc_y, self.ball_loc_z)-93
        self.bot_speed_linear = self.distance(self.bot_speed_x, self.bot_speed_y, self.bot_speed_z, 0, 0, 0)

        # Blue has their goal at -5000 (Y axis) and orange has their goal at 5000 (Y axis). This means that:
        # - Blue is behind the ball if the ball's Y axis is greater than blue's Y axis
        # - Orange is behind the ball if the ball's Y axis is smaller than orange's Y axis

#normal aim
        if (self.index == 0 and self.bot_loc_y < self.ball_loc_y) or (self.index == 1 and self.bot_loc_y > self.ball_loc_y):
            self.aim(self.ball_loc_x, self.ball_loc_y)
        else:
            if self.index == 0:
                # Blue team's goal is located at (0, -5000)
                self.aim(0, -5000)
            else:
                # Orange team's goal is located at (0, 5000)
                self.aim(0, 5000)

        #perfect speed
        if self.ball_speed_z < 0 :
            time_ball_ground=(self.ball_loc_z-92.12849426269531)/-self.ball_speed_z
            perfect_speed = self.distance_car_ball/time_ball_ground
            print(perfect_speed/self.bot_speed_linear)
            if perfect_speed < self.bot_speed_linear:
                self.throttle=-1
            elif perfect_speed/self.bot_speed_linear>2:
                self.boost=True
            else: self.boost=False
        if self.distance(self.bot_loc_x, self.bot_loc_y, 0, self.ball_loc_x, self.ball_loc_y, 0)<self.ball_loc_z:
            self.throttle=0
        #print(self.distance_car_ball/self.distance(self.bot_speed_x,self.bot_speed_y,self.bot_speed_z,self.ball_speed_x,self.ball_speed_y,self.ball_speed_z))



#dodge
        if self.throttle==1:
            if self.distance(self.bot_loc_x, self.bot_loc_y, 0, self.ball_loc_x, self.ball_loc_y, 0) < 500 and self.ball_loc_z <200 and -60<self.angle_car_ball<60:
                    self.dodge()

            if self.bot_ground:
                self.pitch=0
                if self.after_dodge:
                    self.pitch=0
                    self.after_dodge = False
#right side up
        if not self.bot_ground:
            if self.bot_rot_roll >20:
                self.roll = -1
            elif self.bot_rot_roll <-20:
                self.roll = 1
            else: self.roll = -(self.bot_rot_roll/50)
        return [self.throttle, self.steer, self.pitch, self.yaw, self.roll, self.jump, self.boost, self.powerslide]
