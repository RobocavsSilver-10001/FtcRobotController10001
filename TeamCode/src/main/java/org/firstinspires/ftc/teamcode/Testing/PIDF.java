package org.firstinspires.ftc.teamcode.Testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name = "PIDFTest")
@Config
public class PIDF extends OpMode {
    private PIDController controller;
    public static double p = 0.0033, i = 0, d = 0.0001;
    public static double f = 0.055;

    public static int target = 0;

    private final double ticks_in_degree = 700 / 100.0;

    private DcMotorEx ExtendMotor;


    @Override
    public void init() {
        controller = new PIDController(p, i, d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        ExtendMotor = hardwareMap.get(DcMotorEx.class, "ExtendMotor");
        ExtendMotor.setDirection(DcMotorEx.Direction.REVERSE);
    }

    @Override
    public void loop() {
        controller.setPID(p, i, d);
        int extendPos = ExtendMotor.getCurrentPosition();
        double pid = controller.calculate(extendPos, target);
        double power = pid + f;

        ExtendMotor.setPower(power);

        telemetry.addData("pos", extendPos);
        telemetry.addData("target ", target);
        telemetry.update();
    }
}
