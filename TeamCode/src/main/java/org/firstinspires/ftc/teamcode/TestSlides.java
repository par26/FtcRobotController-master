package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

@Config
@com.qualcomm.robotcore.eventloop.opmode.TeleOp
public class TestSlides extends OpMode {




    DcMotorEx leftSlides;
    DcMotorEx rightSlides;

    public static double arm_deadband = 0.05;


    public static double LeftDirection;
    public static double RightDirection;
    @Override
    public void init() {
        leftSlides = hardwareMap.get(DcMotorEx.class, "leftSlide");
        rightSlides = hardwareMap.get(DcMotorEx.class, "rightSlide");

        //telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        leftSlides.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        rightSlides.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        rightSlides.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        leftSlides.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftSlides.setDirection(DcMotor.Direction.REVERSE);
    }

    @Override
    public void loop() {

        double slidePower = Range.clip(gamepad1.right_trigger - gamepad1.left_trigger, -0.5, 0.5);

        if(Math.abs(slidePower) < arm_deadband) {
            leftSlides.setPower(0.0);
            rightSlides.setPower(0.0);
        } else {
            leftSlides.setPower(slidePower);
            rightSlides.setPower(slidePower);
        }

    }
}
