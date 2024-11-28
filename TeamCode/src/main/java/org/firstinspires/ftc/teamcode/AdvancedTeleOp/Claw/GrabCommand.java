package org.firstinspires.ftc.teamcode.AdvancedTeleOp.Claw;

import com.arcrobotics.ftclib.command.CommandBase;

public class GrabCommand extends CommandBase {
    private final GrabSubsystem grabSubsystem;
    private final boolean shouldOpen;

    // Constructor
    public GrabCommand(GrabSubsystem grabSubsystem, boolean shouldOpen) {
        this.grabSubsystem = grabSubsystem;
        this.shouldOpen = shouldOpen;

        // Add the clawSubsystem as a requirement for this command
        addRequirements(grabSubsystem);
    }

    // Execute method that runs the claw control
    @Override
    public void execute() {
        if (shouldOpen) {
            grabSubsystem.openGrabber();
        } else {
            grabSubsystem.closeGrabber();
        }
    }

    // End the command
    @Override
    public void end(boolean interrupted) {
        // Optional: stop or reset the claw if needed after the command ends
    }

    // Is the command finished? It is not finished because we are waiting for gamepad input.
    @Override
    public boolean isFinished() {
        return false;
    }
}
