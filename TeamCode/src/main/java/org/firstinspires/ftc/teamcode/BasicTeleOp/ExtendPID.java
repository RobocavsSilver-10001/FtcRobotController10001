package org.firstinspires.ftc.teamcode.BasicTeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;


@Config
@TeleOp
public class ExtendPID extends OpMode {

    private PIDController controller;
    public static double p = 0.006, i = 0, d = 0.0002;
    public static double f = 0.003;

    public static int target = 0;


    private DcMotorEx extendMotorFront;
    private DcMotorEx extendMotorBack;


    @Override
    public void init() {


        controller = new PIDController(p, i, d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());


        extendMotorFront = hardwareMap.get(DcMotorEx.class, "ExtendMotorFront");
        extendMotorBack = hardwareMap.get(DcMotorEx.class, "ExtendMotorBack");

        extendMotorFront.setDirection(DcMotorEx.Direction.FORWARD);
        extendMotorBack.setDirection(DcMotorSimple.Direction.REVERSE);
    }
    @Override
    public void loop() {
        controller.setPID(p, i, d);
        int extendpos = extendMotorBack.getCurrentPosition();
        double pid = controller.calculate(extendpos, target);
        double power = pid + f;


        extendMotorBack.setPower(power);
        extendMotorFront.setPower(power);


        telemetry.addData("pos", extendpos);
        telemetry.addData("target", target);
        telemetry.update();
    }
}


