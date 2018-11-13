import math
import time
import LinearAlgebra as la
import Simulation as sim

from rlbot.agents.base_agent import BaseAgent, SimpleControllerState
from rlbot.utils.structures.game_data_struct import GameTickPacket
from rlbot.utils.game_state_util import GameState, BallState, CarState, Physics, Vector3, Rotator


class SkyBot(BaseAgent):
    def __init__(self, name, team, index):
        self.index = index
        self.name = name
        self.controller = None
        self.self = None

        # Bot Physics
        self.loc = None
        self.rot = None
        self.theta = None
        self.vel = None
        self.a_vel = None

        # Other bot info
        self.demoed = None
        self.wheel_contact = None
        self.sonic = None
        self.jumped = None
        self.d_jumped = None
        self.team = team
        self.boost = None

        # Ball physics
        self.ball = None
        self.ball_loc = None
        self.ball_rot = None
        self.ball_vel = None

        # Game info
        self.game = None
        self.n_cars = None
        self.s_elapsed = None
        self.r_active = None
        self.in_kickoff = None
        self.ended = None

        # Field info
        self.field = None
        self.own_goal = None
        self.own_goal_loc = None
        self.enemy_goal = None
        self.enemy_goal_loc = None
        self.n_boost_pads = None
        # Normalize boost pads
        self.boost_pads = None

        # Non Packet
        self.state = None
        self.max_speed = [1410, 2300]  # normal, boost
        self.bounce = False

    def initialize_agent(self):
        self.controller = SimpleControllerState()

    def update_values(self, packet):
        self.self = packet.game_cars[self.index]

        # Bot Physics
        self.loc = self.self.physics.location
        self.loc = la.vec3(self.loc.x, self.loc.y, self.loc.z)
        self.rot = self.self.physics.rotation
        self.rot = la.vec3(self.rot.pitch, self.rot.yaw, self.rot.roll)
        self.theta = la.euler_rotation(self.rot)
        self.vel = self.self.physics.velocity
        self.vel = la.vec3(self.vel.x, self.vel.y, self.vel.z)
        self.a_vel = self.self.physics.angular_velocity
        self.a_vel = la.vec3(self.a_vel.x, self.a_vel.y, self.a_vel.z)
        self.max_current_radius = 1 / sim.max_curvature(la.norm(self.vel))

        # Other bot info
        self.demoed = self.self.is_demolished
        self.wheel_contact = self.self.has_wheel_contact
        self.sonic = self.self.is_super_sonic
        self.jumped = self.self.jumped
        self.d_jumped = self.self.double_jumped
        self.team = self.self.team
        self.boost = self.self.boost

        # Ball physics
        self.ball = packet.game_ball
        self.ball_loc = self.ball.physics.location
        self.ball_loc = la.vec3(self.ball_loc.x, self.ball_loc.y, self.ball_loc.z)
        self.ball_rot = self.ball.physics.rotation
        self.ball_rot = la.vec3(self.ball_rot.pitch, self.ball_rot.yaw, self.ball_rot.roll)
        self.ball_vel = self.ball.physics.velocity
        self.ball_vel = la.vec3(self.ball_vel.x, self.ball_vel.y, self.ball_vel.z)

        # Game info
        self.game = packet.game_info
        self.n_cars = packet.num_cars
        self.time = self.game.seconds_elapsed
        self.r_active = self.game.is_round_active
        self.in_kickoff = self.game.is_kickoff_pause
        self.ended = self.game.is_match_ended

        # Field info
        self.field = self.get_field_info()
        for goal in range(self.field.num_goals):
            if self.field.goals[goal].team_num == self.team:
                self.own_goal = self.field.goals[goal]
                self.own_goal_loc = la.vec3(self.own_goal.location.x,
                                            self.own_goal.location.y,
                                            self.own_goal.location.z)
            else:
                self.enemy_goal = self.field.goals[goal]
                self.enemy_goal_loc = la.vec3(self.enemy_goal.location.x,
                                              self.enemy_goal.location.y,
                                              self.enemy_goal.location.z)
        self.n_boost_pads = self.field.num_boosts
        # Normalize boost pads
        self.boost_pads = []  # [(is_active, location, is_full_boost, timer)]
        for pad in range(self.n_boost_pads):
            self.boost_pads = self.boost_pads + [(
                packet.game_boosts[pad].is_active,
                self.field.boost_pads[pad].location,
                self.field.boost_pads[pad].is_full_boost,
                packet.game_boosts[pad].timer)]

    def return_ball_prediction(self, draw=False):
        self.ball_prediction_font = self.get_ball_prediction_struct()
        self.ball_prediction = [[], []]
        self.ball_prediction[0] = [None] * self.ball_prediction_font.num_slices
        self.ball_prediction[1] = [None] * self.ball_prediction_font.num_slices
        if self.ball_prediction_font is not None:
            for p_slice in range(0, self.ball_prediction_font.num_slices):
                prediction_slice = self.ball_prediction_font.slices[p_slice]
                prediction_loc = prediction_slice.physics.location
                self.ball_prediction[0][p_slice] = prediction_slice.game_seconds
                self.ball_prediction[1][p_slice] = la.vec3(
                    prediction_loc.x,
                    prediction_loc.y,
                    prediction_loc.z)
            if draw:
                self.renderer.begin_rendering('Prediction')
                self.renderer.draw_polyline_3d(self.ball_prediction[1], self.renderer.black())
                self.renderer.end_rendering()

    def get_ground_bounces(self):
        self.ground_bounce = [[], []]
        for p_slice in range(1, self.ball_prediction_font.num_slices):
            prev_ang_v = self.ball_prediction_font.slices[p_slice - 1].physics.angular_velocity
            prev_norm_ang_vel = (math.sqrt(prev_ang_v.x ** 2 + prev_ang_v.y ** 2 + prev_ang_v.z ** 2))
            current_slice = self.ball_prediction_font.slices[p_slice]
            current_ang_v = self.ball_prediction_font.slices[p_slice].physics.angular_velocity
            current_norm_ang_vel = (math.sqrt(current_ang_v.x ** 2 + current_ang_v.y ** 2 + current_ang_v.z ** 2))
            if prev_norm_ang_vel != current_norm_ang_vel and current_slice.physics.location.z < 125:
                self.ground_bounce[0] = self.ground_bounce[0] + [current_slice.game_seconds]
                self.ground_bounce[1] = self.ground_bounce[1] + [la.vec3(
                    current_slice.physics.location.x,
                    current_slice.physics.location.y,
                    current_slice.physics.location.z)]
        return

    def get_state(self):
        if self.in_kickoff:
            self.state = 'kickoff'
        elif self.state == 'kickoff':
            self.state = 'go for dribble'
        return

    def g_vec2(self, vec3):
        vec2 = la.vec2(vec3[0], vec3[1])
        return vec2

    def g_vec3(self, vec2):
        vec3 = la.vec3(vec2[0], vec2[1], 0)
        return vec3

    def choose_bounce_new_way(self):
        ball_goal_vector = la.vec2(self.enemy_goal_loc[0], self.enemy_goal_loc[1]) - \
                           la.vec2(self.ball_loc[0], self.ball_loc[1])
        radius_multiplier = 1 / 1  # 0.8
        num_bounce = len(self.ground_bounce[1])
        # num_bounce = 3
        path = [None] * num_bounce
        end_radius = 1 / sim.max_curvature(1000)
        self.renderer.begin_rendering('bounce time')
        for bounce_n in range(num_bounce):
            bounce = self.ground_bounce[1][bounce_n]
            if bounce_n < 3:
                self.renderer.draw_string_3d(bounce, 1, 1, str(self.ground_bounce[0][bounce_n] - self.time),
                                             self.renderer.black())
            bounce = self.grounded(bounce)
            radius_1_sgn = la.sgn(self.ball_loc_loc[1])
            radius_2_sgn = -la.sgn(self.ball_loc_loc[1] * self.enemy_goal_loc_loc[0])
            path[bounce_n] = sim.ArcLineArc(self.g_vec2(self.loc),
                                            self.g_vec2(la.normalize(self.vel)),
                                            self.max_current_radius * radius_1_sgn * radius_multiplier,
                                            self.g_vec2(bounce) + la.normalize(ball_goal_vector) * -300,
                                            la.normalize(ball_goal_vector),
                                            end_radius * radius_2_sgn * radius_multiplier)
            # path.is_valid
            # path.length
            # path.p1  # point 1
            # path.p2  # point 2
            # path.q1  # circle to line 1
            # path.q2  # line to circle 2
            # path.o1  # circle 1 center
            # path.o2  # circle 2 center
            # print(path.q1)
        self.renderer.end_rendering()
        self.renderer.begin_rendering('path')
        for line in range(len(path)):
            speed_needed = abs((path[line].length + 700) / ((self.ground_bounce[0][line]) - self.time))
            if 0 > speed_needed or speed_needed > self.max_speed[1]:
                continue
            else:
                self.renderer.draw_line_3d(self.g_vec3(path[line].q1), self.g_vec3(path[line].q2), self.renderer.pink())
                points = 25
                curve1 = [None] * (points + 1)
                curve2 = [None] * (points + 1)
                for divn in range(points + 1):
                    x = math.cos(((2 * math.pi) / points) * divn) * -self.max_current_radius * radius_multiplier
                    y = math.sin(((2 * math.pi) / points) * divn) * -self.max_current_radius * radius_multiplier
                    x2 = math.cos(((2 * math.pi) / points) * divn) * -end_radius * radius_multiplier
                    y2 = math.sin(((2 * math.pi) / points) * divn) * -end_radius * radius_multiplier
                    curve1[divn] = self.to_world(la.vec3(x, y, 0)) + self.g_vec3(path[line].o1)
                    curve2[divn] = self.to_world(la.vec3(x2, y2, 0)) + self.g_vec3(path[line].o2)
                self.renderer.draw_polyline_3d(curve1, self.renderer.pink())
                self.renderer.draw_polyline_3d(curve2, self.renderer.pink())
                self.renderer.draw_line_3d(self.g_vec3(path[line].p2), self.g_vec3(self.enemy_goal_loc),
                                           self.renderer.pink())
                self.renderer.end_rendering()
                return [self.g_vec3(path[line].q2), speed_needed]
        self.renderer.end_rendering()
        return self.choose_bounce_old_way()

    def choose_bounce_old_way(self):
        ball_goal_vector = la.vec2(self.enemy_goal_loc[0], self.enemy_goal_loc[1]) - \
                           la.vec2(self.ball_loc[0], self.ball_loc[1])
        radius_multiplier = 1 / 1  # 0.8
        num_bounce = len(self.ground_bounce[1])
        # num_bounce = 3
        path = [None] * num_bounce
        for bounce_n in range(num_bounce):
            bounce = self.ground_bounce[1][bounce_n]
            if ((self.ground_bounce[0][bounce_n]) - self.time) != 0:
                speed_needed = la.norm(bounce - self.loc) / ((self.ground_bounce[0][bounce_n]) - self.time)
            else:
                speed_needed = self.max_speed[1]
            if 0 > speed_needed or speed_needed > self.max_speed[1]:
                continue
            else:
                return [bounce + (self.g_vec3(la.normalize(ball_goal_vector)) * -40), speed_needed]
        return [self.ball_loc, self.time+0.01]

    def what_to_do(self):
        # todo: add  ball saving state
        if self.state == None:
            self.state = 'go for dribble'
        if self.state == 'kickoff':
            self.controller.boost = 1
            self.controller.throttle = 1
            ball_loc_ground = self.grounded(self.ball_loc)
            self.aim(self.to_local(ball_loc_ground - self.loc))
        elif self.state == 'go for dribble' or self.state == 'going for dribble':
            if self.state == 'go for dribble' or self.bounce[0] == None:
                self.state = 'going for dribble'
                self.bounce = self.choose_bounce_new_way()
            if self.bounce[1] < self.time:
                self.state = 'go for dribble'
            if self.state == 'going for dribble' and la.norm(self.bounce[0] - self.loc) < 100:
                self.state = 'dribble'
            self.aim(self.to_local(self.bounce[0] - self.loc), speed_needed=self.bounce[1])
        elif self.state == 'dribble':
            if (la.norm(self.ball_loc_loc) > (5 * self.max_current_radius)) and math.atan(self.to_local(self.bounce[0] - self.loc)[1] / self.to_local(self.bounce[0] - self.loc)[0]) > math.pi:
                self.state = 'go for dribble'
            self.bounce = self.choose_bounce_old_way()
            self.aim(self.to_local(self.bounce[0] - self.loc), speed_needed=self.bounce[1])

    def render(self):
        self.renderer.begin_rendering('stuff')
        # AXIS
        # X - CYAN
        self.renderer.draw_line_3d((self.to_world(la.vec3(100, 0, 0)) + self.loc), self.loc, self.renderer.cyan())
        # Y - RED
        self.renderer.draw_line_3d((self.to_world(la.vec3(0, 100, 0)) + self.loc), self.loc, self.renderer.red())
        # X - GREEN
        self.renderer.draw_line_3d((self.to_world(la.vec3(0, 0, 100)) + self.loc), self.loc, self.renderer.green())

        # car to ball
        # self.renderer.draw_line_3d(self.ball_loc, self.loc, self.renderer.team_color())

        # car to goal corners
        self.renderer.draw_line_3d(self.enemy_goal_loc + la.vec3(892.755, 0, 642.775 / 2), self.loc,
                                   self.renderer.team_color(1 - self.team))
        self.renderer.draw_line_3d(self.enemy_goal_loc + la.vec3(892.755, 0, -642.775 / 2), self.loc,
                                   self.renderer.team_color(1 - self.team))
        self.renderer.draw_line_3d(self.enemy_goal_loc + la.vec3(-892.755, 0, 642.775 / 2), self.loc,
                                   self.renderer.team_color(1 - self.team))
        self.renderer.draw_line_3d(self.enemy_goal_loc + la.vec3(-892.755, 0, -642.775 / 2), self.loc,
                                   self.renderer.team_color(1 - self.team))

        # curvature radius
        points = 25
        curve = [None] * (points + 1)
        both_sides = [-1, 1]
        for side in both_sides:
            for divn in range(points + 1):
                x = math.cos(((2 * math.pi) / points) * divn) * self.max_current_radius
                y = math.sin(((2 * math.pi) / points) * divn) * self.max_current_radius
                curve[divn] = self.to_world(la.vec3(x, y, 0) + la.vec3(0, self.max_current_radius * side, 0)) + self.loc
            self.renderer.draw_polyline_3d(curve, self.renderer.team_color())
        points = 100
        curve = [None] * (points + 1)
        for divn in range(points + 1):
            x = math.cos(((2 * math.pi) / points) * divn) * self.max_current_radius * (1 / self.controller.steer)
            y = math.sin(((2 * math.pi) / points) * divn) * self.max_current_radius * (1 / self.controller.steer)
            curve[divn] = self.to_world(
                la.vec3(x, y, 0) + la.vec3(0, self.max_current_radius * (1 / self.controller.steer), 0)) + self.loc
        self.renderer.draw_polyline_3d(curve, self.renderer.black())

        self.renderer.end_rendering()
        return

    def aim(self, local_target, speed_needed=99999):
        self.renderer.begin_rendering('aim')
        self.renderer.draw_line_3d((self.to_world(local_target) + self.loc), self.loc, self.renderer.green())
        self.renderer.end_rendering()

        alpha = math.atan(local_target[1] / local_target[0])
        min_angle = 10
        if math.sin(local_target[0] / la.norm(local_target)) < 0:
            alpha = -alpha
        if alpha < math.radians(-min_angle):
            # If the target is more than 10 degrees right from the centre, steer left
            self.controller.steer = -1
        elif alpha > math.radians(min_angle):
            # If the target is more than 10 degrees left from the centre, steer right
            self.controller.steer = 1
        else:
            # If the target is less than 10 degrees from the centre, steer straight
            self.controller.steer = math.degrees(alpha) / min_angle

        if speed_needed > 0:
            if speed_needed > self.max_speed[0]:
                self.controller.throttle = 1
                self.controller.boost = 1
            elif la.norm(self.vel) < speed_needed:
                self.controller.throttle = 1
                self.controller.boost = 0
                if la.norm(local_target) < 200:
                    self.controller.throttle = la.norm(local_target) / 200
        if speed_needed < la.norm(self.vel):
            self.controller.throttle = -1
            self.controller.boost = 0
            if la.norm(local_target) < 200:
                self.controller.throttle = -la.norm(local_target) / 200
            if la.norm(local_target) < 100:
                self.controller.throttle = -la.norm(local_target) / 100
        if self.sonic:
            self.controller.boost = 0
        return


    def to_local(self, vector):
        return la.dot(vector, self.theta)


    def to_world(self, vector):
        return la.dot(self.theta, vector)


    def get_local(self):
        self.ball_loc_loc = self.to_local(self.ball_loc - self.loc)
        self.ball_vel_loc = self.to_local(self.ball_vel)
        self.enemy_goal_loc_loc = self.to_local(self.enemy_goal_loc - self.loc)
        return


    def grounded(self, vector):
        vector2 = la.vec3(0, 0, 0)
        vector2[0] = vector[0]
        vector2[1] = vector[1]
        vector2[2] = 0
        return vector2


    def set_state(self):
        number = int(self.time) % 20
        enemy_state = CarState(physics=Physics(location=Vector3(x=0, y=self.enemy_goal_loc[1], z=100),
                                               rotation=Rotator(0, 0, 0)))

        car_state = CarState(boost_amount=100, physics=Physics(location=Vector3(x=0, y=-500, z=0),
                                                               rotation=Rotator(0, math.pi / 2, 0),
                                                               velocity=Vector3(x=0, y=300, z=0)))
        ball_state = BallState(Physics(location=Vector3(-2000, -5000, 1000), velocity=Vector3(x=0, y=500, z=200)))
        game_state = GameState(ball=ball_state, cars={self.index: car_state, 1: enemy_state})
        game_state2 = GameState(cars={1: enemy_state})
        # game_state = GameState(cars={self.index: car_state})
        if not self.in_kickoff:
            if number == 0:
                self.set_game_state(game_state)
            else:
                self.set_game_state(game_state2)


    def get_output(self, packet: GameTickPacket) -> SimpleControllerState:
        self.update_values(packet)
        if not self.ended and self.r_active:
            self.get_local()
            self.get_state()
            self.return_ball_prediction(draw=False)
            self.get_ground_bounces()

            self.what_to_do()
            print(self.state)
            print(self.in_kickoff)

            # self.render()

            # self.set_state()

            return self.controller
