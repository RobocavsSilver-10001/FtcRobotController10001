/*package org.firstinspires.ftc.teamcode.AdvancedOpModes.ArmAngle;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;

import org.firstinspires.ftc.teamcode.AdvancedOpModes.Robot.RobotHardware;

public class AngleSubsystem extends SubsystemBase {

    private final RobotHardware robotHardware;
    private final PIDController controller;

    // Constants for PID and Feedforward
    private static final double P = 0.0041;
    private static final double I = 0;
    private static final double D = 0.00015;
    private static final double F = 0.12;

    // Angle limits (in encoder ticks)
    public static final int MIN_ANGLE = -825;
    public static final int MAX_ANGLE = 0;  // Update as per your robot's limits

    // Offset and other parameters
    private static final double OFFSET = Math.PI / 3.6;  // Radians
    private static final double TICKS_IN_DEGREES = 700 / 180.0;  // Conversion factor from degrees to ticks

    private int targetAngle = -825;  // Target position (in degrees)

    public AngleSubsystem(RobotHardware robotHardware) {
        this.robotHardware = robotHardware;
        this.controller = new PIDController(P, I, D);
    }

    // Set target angle
    public void setArmAngle(int targetAngle) {
        this.targetAngle = targetAngle;
    }

    // Update method to be called in the teleop loop
    public void update() {
        int armPos = robotHardware.leftAngle.getCurrentPosition();  // Get current arm position

        // Calculate the PID output
        double pidOutput = controller.calculate(armPos, targetAngle);

        // Feedforward term for smooth motion with gravity compensation
        // Assumed feedforward calculation, adjust based on your robot's actual setup
        double ffOutput = Math.cos(Math.toRadians(targetAngle / TICKS_IN_DEGREES) - OFFSET) * F;

        // Final power to apply to the motors
        double power = pidOutput + ffOutput;

        // Apply power to both motors (assuming both sides are responsible for moving the arm)
        robotHardware.leftAngle.setPower(power);
        robotHardware.rightAngle.setPower(power);
    }

    @Override
    public void periodic() {
        update();  // Continuously update the arm control
    }
}
*/