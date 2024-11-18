package org.firstinspires.ftc.teamcode.AdvancedTeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.AdvancedTeleOp.ArmAngle.AngleCommand;
import org.firstinspires.ftc.teamcode.AdvancedTeleOp.ArmAngle.AngleSubsystem;
import org.firstinspires.ftc.teamcode.AdvancedTeleOp.Robot.RobotHardware;
import org.firstinspires.ftc.teamcode.AdvancedTeleOp.DriveBase.MecanumDrive;

@TeleOp(name = "AdvancedTeleOp")
public class CommandTeleOp extends CommandOpMode {

    // Hardware instance
    private final RobotHardware robot = RobotHardware.getInstance();

    // Gamepad instances
    private GamepadEx gamepadex1;
    private GamepadEx gamepadex2;

    // Drive subsystem instance
    private MecanumDrive mecanumDrive;

    // Arm subsystem instance
    private AngleSubsystem armSubsystem;

    // Variables for arm control
    private int armTargetAngle = -650; // Initial position

    @Override
    public void initialize() {

        CommandScheduler.getInstance().reset();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // Initialize hardware components
        robot.init(hardwareMap);

        // Initialize the drive system
        mecanumDrive = new MecanumDrive(robot);

        // Initialize the arm subsystem
        armSubsystem = new AngleSubsystem(robot);

        // Initialize gamepads
        gamepadex1 = new GamepadEx(gamepad1);
        gamepadex2 = new GamepadEx(gamepad2);
    }

    @Override
    public void run() {
        CommandScheduler.getInstance().run();

        // Drive control: Use gamepad1 to control the mecanum drive
        mecanumDrive.drive(gamepadex1);

        // Arm control via gamepad2 D-pad
        if (gamepadex2.getButton(GamepadKeys.Button.DPAD_UP)) {
            // Move arm up if D-pad up is pressed
            armTargetAngle = Math.min(armTargetAngle + 150, AngleSubsystem.MAX_ANGLE);
        } else if (gamepadex2.getButton(GamepadKeys.Button.DPAD_DOWN)) {
            // Move arm down if D-pad down is pressed
            armTargetAngle = Math.max(armTargetAngle - 150, AngleSubsystem.MIN_ANGLE);
        }


        // Command to set the arm angle
        new AngleCommand(armSubsystem, armTargetAngle).schedule();

        // Update the arm subsystem periodically
        armSubsystem.periodic();

        int armPos = robot.leftAngle.getCurrentPosition();  // Get current arm position
        telemetry.addData("Arm Position", armPos);          // Current arm position
        telemetry.addData("Target Angle", armTargetAngle);  // Desired target angle
        telemetry.update();  // Make sure telemetry is updated
    }
}
