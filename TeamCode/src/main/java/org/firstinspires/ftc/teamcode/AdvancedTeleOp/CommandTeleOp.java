package org.firstinspires.ftc.teamcode.AdvancedTeleOp;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.AdvancedTeleOp.Robot.RobotHardware;

@TeleOp(name = "AdvancedTeleOp")
public class CommandTeleOp extends CommandOpMode {

    private final RobotHardware robot = RobotHardware.getInstance();
    private GamepadEx gamepadex1;
    private GamepadEx gamepadex2;

    @Override
    public void initialize() {
        CommandScheduler.getInstance().reset();

    }
}
