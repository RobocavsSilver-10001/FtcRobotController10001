package org.firstinspires.ftc.teamcode.BasicTeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Config
@TeleOp
public class DynamicPIDF extends OpMode {

    private PIDController controller;
    public static double p = 0.0075, i = 0, d = 0.0005;
    public static double f = 0.1;

    public static int target = 0;


    private final double ticks_in_degrees = 700/180.0;
    private DcMotorEx angMotorLeft;
    private DcMotorEx angMotorRight;


    @Override
    public void init() {


        controller = new PIDController(p, i, d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());


        angMotorLeft = hardwareMap.get(DcMotorEx.class, "AngleMotorLeft");
        angMotorRight = hardwareMap.get(DcMotorEx.class, "AngleMotorRight");

        angMotorLeft.setDirection(DcMotorEx.Direction.REVERSE);
        angMotorRight.setDirection(DcMotorSimple.Direction.FORWARD);
    }
    @Override
    public void loop() {
        controller.setPID(p, i, d);
        int armpos = angMotorLeft.getCurrentPosition();

        // Convert current position from ticks to degrees
        double currentAngle = armpos / ticks_in_degrees;

        double pid = controller.calculate(armpos, target);

        // Adjust feedforward strength based on the angle
        double ff = Math.cos(Math.toRadians(currentAngle)) * f;

        // Combine PID and feedforward terms
        double power = pid + ff;

        angMotorLeft.setPower(power);
        angMotorRight.setPower(power);

        telemetry.addData("pos", armpos);
        telemetry.addData("target", target);
        telemetry.addData("currentAngle", currentAngle); // Optional: for debugging
        telemetry.addData("feedforward", ff); // Optional: for debugging
        telemetry.update();
    }

}


