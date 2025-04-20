import pymunk
import pyglet

from pyglet.window import key
from pymunk import Vec2d
from pymunk.pyglet_util import DrawOptions


class Pendulum:

    MASS = 0.100  # kg
    FORCE = 10  # mN

    def __init__(self, space: pymunk.Space):
        self.space = space

        self._create_entities()

    def _create_entities(self) -> None:
        """Create the entities that form the Pendulum."""
        self.static_body = pymunk.Body(body_type=pymunk.Body.STATIC)
        self.static_body.position = (360, 360)

        moment = pymunk.moment_for_circle(
            mass=self.MASS, inner_radius=0, outer_radius=10.0
        )
        self.circle_body = pymunk.Body(mass=self.MASS, moment=moment)
        self.circle_body.position = (360, 50)

        circle_shape = pymunk.Circle(body=self.circle_body, radius=10.0)

        rod_joint = pymunk.constraints.PinJoint(
            a=self.static_body,
            b=self.circle_body,
        )

        self.space.add(
            self.static_body, self.circle_body, circle_shape, rod_joint
        )

    @property
    def angle(self) -> float:
        """Angle (deg) between the Pendulum and the resting location."""
        return Vec2d(0, -1).get_angle_degrees_between(self.vector)

    @property
    def vector(self) -> Vec2d:
        """Pendulum Vector, from Fixed point to the center of the Circle."""
        return self.circle_body.position - self.static_body.position

    def accelerate(self, direction: Vec2d):
        """Apply force in the direction `dir`."""
        impulse = self.FORCE * direction.normalized()
        self.circle_body.apply_impulse_at_local_point(impulse=impulse)


class SimulationWindow(pyglet.window.Window):
    """Application simulating the Fixed Pendulum."""

    CAPTION = "PyMunk Fixed Pendulum Simulation."

    WIDTH = 720
    HEIGHT = 720

    INTERVAL = 1.0 / 100 

    FONT_SIZE = 16
    FONT_COLOR = (255, 255, 255, 255)

    def __init__(self):
        super().__init__(
            width=self.WIDTH, height=self.HEIGHT, caption=self.CAPTION
        )

        self.space = pymunk.Space()
        self.space.gravity = Vec2d(0, -9807)  # mm/s²

        self.model = Pendulum(space=self.space)

        self.draw_options = DrawOptions()
        self.keyboard = key.KeyStateHandler()
        self.push_handlers(self.keyboard)

        self.angle_label = pyglet.text.Label(
            font_size=self.FONT_SIZE,
            x=5,
            y=self.height - self.FONT_SIZE - 1,
            color=self.FONT_COLOR,
        )

        pyglet.clock.schedule_interval(self.update, interval=self.INTERVAL)

    def _handle_input(self):
        """Handle application input."""
        if self.keyboard[key.LEFT]:
            direction = self.model.vector.rotated_degrees(-90)  # CW
            self.model.accelerate(direction=direction)
        elif self.keyboard[key.RIGHT]:
            direction = self.model.vector.rotated_degrees(90)  # CCW
            self.model.accelerate(direction=direction)

    def on_draw(self) -> None:
        """Screen Draw Event."""
        self.clear()
        self.space.debug_draw(options=self.draw_options)
        self.angle_label.draw()

    def update(self, dt: float) -> None:
        """Update PyMunk's Space state.
        :param float dt: Time between calls of `update`.
        """
        self._handle_input()
        self.space.step(dt=self.INTERVAL)
        self.angle_label.text = f"Angle: {self.model.angle:4.0f}°"


if __name__ == "__main__":
    SimulationWindow()
    pyglet.app.run()