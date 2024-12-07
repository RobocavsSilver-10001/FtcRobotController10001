/*package org.firstinspires.ftc.teamcode.AdvancedOpModes.ArmAngle;

import com.arcrobotics.ftclib.command.CommandBase;

public class AngleCommand extends CommandBase {

    private final AngleSubsystem angleSubsystem;
    private final int targetAngle;

    public AngleCommand(AngleSubsystem angleSubsystem, int targetAngle) {
        this.angleSubsystem = angleSubsystem;
        this.targetAngle = targetAngle;
    }

    @Override
    public void initialize() {
        // Set the target angle when the command is initialized
        angleSubsystem.setArmAngle(targetAngle);
    }

    @Override
    public void execute() {
        // Continuously update the arm during command execution
        // The angleSubsystem's update will control the arm using PIDF and hold the position
        angleSubsystem.periodic();
    }

    @Override
    public boolean isFinished() {
        // The command is never finished because we want the arm to keep holding the position
        // You could add logic to end the command if you want, such as after reaching a certain angle
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        // Optionally handle the end of the command (e.g., stop motors)
        if (interrupted) {
            angleSubsystem.setArmAngle(-825);  // Stop the arm if interrupted, or set to default position
        }
        else {
            interrupted = false;
        }
    }
}
*/