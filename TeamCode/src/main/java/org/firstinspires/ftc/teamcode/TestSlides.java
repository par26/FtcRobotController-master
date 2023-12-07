package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp
public class TestSlides extends TeleOp{


    DcMotorEx leftSlides;

    double motorDirection = 1.0;
    @Override
    public void init() {
        leftSlides = hardwareMap.get(DcMotorEx.class, "leftSlide");

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        leftSlides.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        motorDirection = 1.0;
    }

    @Override
    public void loop() {


        if(gamepad1.b) {
            motorDirection *= -1;
        }
        double slidePower = gamepad1.right_trigger - gamepad1.left_trigger;
        leftSlides.setPower((slidePower*motorDirection));


        telemetry.addData("pos ", leftSlides.getCurrentPosition());
        telemetry.addData("direction ", motorDirection);
        telemetry.update();

    }
}
