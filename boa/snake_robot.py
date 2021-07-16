import math
import pymunk
import pymunk.pygame_util


class SnakeLink:
    def __init__(self, link_number: int, length: float, radius: float, mass: float, position: (float, float)):
        inertia = pymunk.moment_for_segment(
            mass=mass,
            a=(-length / 2, 0),
            b=(length / 2, 0),
            radius=radius
        )
        self.body = pymunk.Body(mass, inertia)
        self.body.position = position

        self.shape = pymunk.Segment(
            self.body,
            a=(-length / 2, 0),
            b=(length / 2, 0),
            radius=radius,
        )
        self.shape.elasticity = 0.5
        self.shape.collision_type = link_number

        self.link_number = link_number
        self.collision_force = pymunk.Vec2d(0, 0)

    def add_to_space(self, space: pymunk.Space):
        collision_handler = space.add_collision_handler(0, self.link_number)
        collision_handler.post_solve = self.collision_handler
        collision_handler.separate = self.on_collision_separate
        space.add(self.body, self.shape)

    def collision_handler(self, arbiter: pymunk.Arbiter, _space, _data):
        self.collision_force = arbiter.total_impulse

    def on_collision_separate(self, _arbiter, _space, _data):
        self.collision_force = pymunk.Vec2d(0, 0)

    def remove_from_space(self, space: pymunk.Space):
        space.remove(self.body, self.shape)


class SnakeJoint:
    def __init__(self, link_a: pymunk.Body, link_b: pymunk.Body, position: (float, float)):
        self._joint = pymunk.PivotJoint(link_a, link_b, position)
        self._limiter = pymunk.RotaryLimitJoint(link_a, link_b, min=-math.pi / 2, max=math.pi / 2)
        self._motor = pymunk.SimpleMotor(link_a, link_b, rate=0)
        self._motor.collide_bodies = False
        self._motor.max_force = 500_000

    def set_speed(self, speed):
        self._motor.rate = speed

    def add_to_space(self, space: pymunk.Space):
        space.add(self._joint, self._limiter, self._motor)

    def remove_from_space(self, space: pymunk.Space):
        space.remove(self._joint, self._limiter, self._motor)


class SnakeRobot:
    MAX_JOINT_SPEED = 1

    def __init__(self, n_links: int, link_mass: float, radius: float, link_length: float,
                 position: (float, float) = (0, 0)):

        x_init, y_init = position

        self.n_links = n_links
        self._links: [SnakeLink] = []
        self._joints: [SnakeJoint] = []

        for n in range(n_links):
            self._links.append(SnakeLink(
                link_number=n+1,
                length=link_length,
                mass=link_mass,
                radius=radius,
                position=(x_init + n * link_length, y_init)
            ))

        for n in range(n_links-1):
            link_a, link_b = self._links[n].body, self._links[n + 1].body
            self._joints.append(SnakeJoint(
                link_a=link_a,
                link_b=link_b,
                position=(x_init + link_length / 2 + n * link_length, y_init)
            ))
        self.head = self._links[-1]

    def add_to_space(self, space: pymunk.Space):
        for link in self._links:
            link.add_to_space(space)

        for joint in self._joints:
            joint.add_to_space(space)

    def remove_from_space(self, space: pymunk.Space):
        for link in self._links:
            link.remove_from_space(space)

        for joint in self._joints:
            joint.remove_from_space(space)

    def set_motor_speeds(self, speed_rates: [float]):
        for joint, speed_rate in zip(self._joints, speed_rates):
            joint.set_speed(speed_rate*self.MAX_JOINT_SPEED)

    def get_link_angles(self):
        return [link.body.angle for link in self._links]

    def get_joint_angles(self):
        link_angles = self.get_link_angles()
        angle_pairs = zip(link_angles[:-1], link_angles[1:])
        return [b - a for a, b in angle_pairs]

    def get_link_angvels(self):
        return [link.body.angular_velocity for link in self._links]

    def get_collision_forces(self):
        return [link.collision_force for link in self._links]