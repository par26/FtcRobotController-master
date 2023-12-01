package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.drive.Drive;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class TestTeleop extends OpMode {

    MechanumDrive drive = new MechanumDrive();

    DcMotorEx leftSlide;
    final double armManualDeadband = 0.03;

    @Override
    public void init() {

        drive.init(hardwareMap);

        leftSlide = hardwareMap.get(DcMotorEx.class, "leftSlideMotor");
        leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    @Override
    public void loop() {
        double forward = drive.squareInput(-gamepad1.left_stick_y * 1.2);
        double rotate = drive.squareInput(gamepad1.left_stick_x * .8);

        double right = -drive.squareInput(gamepad1.right_stick_x);

        drive.drive(forward, right, rotate);

        double slidePower = gamepad1.right_trigger -  gamepad1.left_trigger;
        //drive.driveFieldRelative(forward, right, rotate);
        if (Math.abs(slidePower) > armManualDeadband) {
            leftSlide.setPower(slidePower);
        }

    }
}
