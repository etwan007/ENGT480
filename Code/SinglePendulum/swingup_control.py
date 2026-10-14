"""Acceleration-based energy pumping with a nonblocking pause at swing reversals."""

import math


def clip(value, limit):
    return max(-limit, min(limit, value))


class SwingUpController:
    def __init__(self, *, inertia, mgl, energy_gain=10.0, max_acceleration=2.0,
                 max_speed=0.40, center_gain=0.5, hesitation_s=0.10,
                 reversal_speed=0.4, kick_s=0.15):
        self.inertia = inertia
        self.mgl = mgl
        self.energy_gain = energy_gain
        self.max_acceleration = max_acceleration
        self.max_speed = max_speed
        self.center_gain = center_gain
        self.hesitation_s = hesitation_s
        self.reversal_speed = reversal_speed
        self.kick_s = kick_s
        self.reset()

    def reset(self, velocity=0.0, *, startup=True):
        self.velocity = clip(velocity, self.max_speed)
        self.direction = 0
        self.phase = "pump"
        self.hold_remaining = 0.0
        self.kick_remaining = self.kick_s if startup else 0.0

    def update(self, theta, omega, cart, dt, *, valid_measurement=True):
        """Return cart velocity; dt is wall-clock elapsed time, in seconds.

        The script's model gives E_dot = mgl/g * omega*cos(theta)*cart_accel.
        Apply the energy law to acceleration, then integrate to motor velocity.
        A reversal brakes the cart and holds it briefly before the next pump.
        No sleep is used, so packet handling and shutdown remain responsive.
        """
        if not math.isfinite(dt) or dt <= 0:
            raise ValueError("Swing-up dt must be finite and positive")
        # Never use a delayed loop to command a large velocity jump.
        dt = min(dt, 0.03)
        if not valid_measurement:
            # A single missing reading must not invent a turn, kick, or catch.
            # Existing packet handling stops the run on the second miss.
            return self.velocity

        direction = (1 if omega > self.reversal_speed else
                     -1 if omega < -self.reversal_speed else 0)
        if direction:
            if (self.direction and direction != self.direction
                    and abs(theta) > 1.0 and self.hesitation_s > 0):
                self.phase = "brake"
                self.hold_remaining = self.hesitation_s
            self.direction = direction

        if self.phase == "brake":
            self.velocity += clip(-self.velocity, self.max_acceleration * dt)
            if abs(self.velocity) < 1e-12:
                self.velocity = 0.0
                self.phase = "hesitate"
            return self.velocity
        if self.phase == "hesitate":
            self.hold_remaining = max(0.0, self.hold_remaining - dt)
            if self.hold_remaining == 0:
                self.phase = "pump"
            return self.velocity

        energy = 0.5 * self.inertia * omega**2 + self.mgl * math.cos(theta)
        acceleration = (self.energy_gain * (self.mgl - energy)
                        * omega * math.cos(theta) - self.center_gain * cart)
        if self.kick_remaining > 0:
            # Break the zero-speed hanging equilibrium once at startup.
            if abs(theta) > math.pi - 0.15 and abs(omega) < self.reversal_speed:
                acceleration = self.max_acceleration
                self.kick_remaining = max(0.0, self.kick_remaining - dt)
            else:
                self.kick_remaining = 0.0
        self.velocity = clip(self.velocity + clip(acceleration, self.max_acceleration) * dt,
                             self.max_speed)
        return self.velocity
