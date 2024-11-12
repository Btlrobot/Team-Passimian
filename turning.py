def turn_pid(self, target_angle, max_speed, kp=0.2, ki=0.0, kd=0.1):
    """
    Turns the robot to a specified angle using PID control.
    Args:
        target_angle (float): Desired angle to turn (degrees, positive for right, negative for left).
        max_speed (int): Maximum motor speed (percent).
        kp (float): Proportional gain.
        ki (float): Integral gain.
        kd (float): Derivative gain.
    """
    self.running = True
    integral = 0
    last_error = 0

    # Reset the gyro to 0 degrees
    self.gyro_3.set_rotation(0, DEGREES)

    while self.running:
        # Get the current rotation
        current_rotation = self.gyro_3.rotation(DEGREES)

        # Calculate the error
        error = target_angle - current_rotation

        # Stop condition: If within a small error margin
        if abs(error) <= 1.0:  # Deadband of ±1 degree
            break

        # PID calculations
        integral += error
        derivative = error - last_error
        correction = kp * error + ki * integral + kd * derivative
        last_error = error

        # Limit correction to max_speed
        correction = max(min(correction, max_speed), -max_speed)

        # Set motor speeds for turning
        left_speed = correction
        right_speed = -correction

        # Clamp motor speeds to max_speed
        left_speed = max(min(left_speed, max_speed), -max_speed)
        right_speed = max(min(right_speed, max_speed), -max_speed)

        # Debugging output
        print(f"Target Angle: {target_angle}, Current Rotation: {current_rotation:.2f}, Error: {error:.2f}")
        print(f"Correction: {correction:.2f}, Left Speed: {left_speed:.2f}, Right Speed: {right_speed:.2f}")

        # Set motor velocities and spin motors
        self.leftMotor.set_velocity(left_speed, PERCENT)
        self.rightMotor.set_velocity(right_speed, PERCENT)
        self.leftMotor.spin(FORWARD)
        self.rightMotor.spin(FORWARD)

        # Small delay for stabilization
        wait(0.02, SECONDS)

    # Stop the motors with braking
    self.leftMotor.stop(BRAKE)
    self.rightMotor.stop(BRAKE)
