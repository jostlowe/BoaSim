import math
import pymunk
import pymunk.pygame_util


class SnakeLink:
    def __init__(self, length: float, radius: float, mass: float, position: (float, float)):
        inertia = pymunk.moment_for_segment(
            mass=mass,
            a=(-length / 2, 0),
            b=(length / 2, 0),
            radius=radius
        )
        self._body = pymunk.Body(mass, inertia)
        self._body.position = position

        self._shape = pymunk.Segment(
            self._body,
            a=(-length / 2, 0),
            b=(length / 2, 0),
            radius=radius
        )
        self._shape.elasticity = 0.5

    def add_to_space(self, space: pymunk.Space):
        space.add(self._body, self._shape)

    def remove_from_space(self, space: pymunk.Space):
        space.remove(self._body, self._shape)


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
    def __init__(self, n_links: int, link_mass: float, radius: float, link_length: float,
                 position: (float, float) = (0, 0)):

        x_init, y_init = position

        self.n_links = n_links
        self._links: [SnakeLink] = []
        self._joints: [SnakeJoint] = []

        for n in range(n_links + 1):
            self._links.append(SnakeLink(
                length=link_length,
                mass=link_mass,
                radius=radius,
                position=(x_init + n * link_length, y_init)
            ))

        for n in range(n_links):
            link_a, link_b = self._links[n]._body, self._links[n + 1]._body
            self._joints.append(SnakeJoint(
                link_a=link_a,
                link_b=link_b,
                position=(x_init + link_length / 2 + n * link_length, y_init)
            ))

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

    def set_motor_speeds(self, speeds: [float]):
        for joint, speed in zip(self._joints, speeds):
            joint.set_speed(speed)

    def get_link_angles(self):
        return [link._body.angle for link in self._links]

    def get_joint_angles(self):
        link_angles = self.get_link_angles()
        angle_pairs = zip(link_angles[:-1], link_angles[1:])
        return [b - a for a, b in angle_pairs]

    def get_heading(self):
        return self._links[-1]._body.angle

    def get_link_angvels(self):
        return [link._body.angular_velocity for link in self._links]

    def get_joint_angvels(self):
        link_angvels = self.get_link_angvels()
        angvel_pairs = zip(link_angvels[:-1], link_angvels[1:])
        return [b - a for a, b in angvel_pairs]

    def get_head_x_vel(self):
        return self._links[-1]._body.velocity.x
