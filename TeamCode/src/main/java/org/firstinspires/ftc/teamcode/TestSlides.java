package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Config
@com.qualcomm.robotcore.eventloop.opmode.TeleOp
public class TestSlides extends TeleOp{




    DcMotorEx leftSlides;
    DcMotorEx rightSlides;

    double motorDirection = 1.0;

    FtcDashboard dashboard;


    public static double Direction;
    @Override
    public void init() {
        leftSlides = hardwareMap.get(DcMotorEx.class, "leftSlide");
        rightSlides = hardwareMap.get(DcMotorEx.class, "rightSlide");

        //telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        leftSlides.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        Direction = 0;

        dashboard = FtcDashboard.getInstance();

        motorDirection = 1.0;
    }

    @Override
    public void loop() {



        //double slidePower = gamepad1.right_trigger - gamepad1.left_trigger;
        leftSlides.setPower(Direction);


        telemetry.addData("pos ", leftSlides.getCurrentPosition());
        telemetry.addData("direction ", motorDirection);
        telemetry.update();

    }
}
