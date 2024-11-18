package org.firstinspires.ftc.teamcode.AdvancedTeleOp.DriveBase;

import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import org.firstinspires.ftc.teamcode.AdvancedTeleOp.Robot.RobotHardware;
import com.arcrobotics.ftclib.gamepad.GamepadEx;

public class MecanumDrive {

    private final RobotHardware robotHardware;
    private final double slowModeFactor = 0.5; // Slow mode speed factor (50% of normal speed)

    public MecanumDrive(RobotHardware robotHardware) {
        this.robotHardware = robotHardware;
    }

    // Method to drive the robot using mecanum controls
    public void drive(GamepadEx gamepad) {
        // Get gamepad values
        double y = -gamepad.getLeftY(); // Forward/backward
        double x = gamepad.getLeftX() * 1.1; // Strafing (left/right)
        double rx = gamepad.getRightX(); // Rotation

        // Denominator ensures that motor speeds are balanced and does not exceed 1
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);

        // Calculate motor powers for each wheel
        double frontLeftPower = (y + x + rx) / denominator;
        double backLeftPower = (y - x + rx) / denominator;
        double frontRightPower = (y - x - rx) / denominator;
        double backRightPower = (y + x - rx) / denominator;

        // Set motor powers using the powerAdjustment method to handle slow mode and any other adjustments
        robotHardware.frontLeft.setPower(powerAdjustment(frontLeftPower, gamepad));
        robotHardware.backLeft.setPower(powerAdjustment(backLeftPower, gamepad));
        robotHardware.frontRight.setPower(powerAdjustment(frontRightPower, gamepad));
        robotHardware.backRight.setPower(powerAdjustment(backRightPower, gamepad));
    }

    // Helper method to adjust the power based on slow mode or other needs
    private double powerAdjustment(double power, GamepadEx gamepad) {
        // Check if slow mode is activated via left bumper
        if (gamepad.getButton(GamepadKeys.Button.LEFT_BUMPER)) {
            power *= slowModeFactor; // Reduce power for slow mode
        }

        return power;
    }
}
