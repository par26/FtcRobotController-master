package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp
public class TeleOp extends OpMode {

    //ProgrammingBoard board = new ProgrammingBoard();
    MechanumDrive drive = new MechanumDrive();
    @Override
    public void init() {
        drive.init(hardwareMap);
    }

    @Override
    public void loop() {
        double forward = -gamepad1.left_stick_y;
        double right = gamepad1.left_stick_x;

        double rotate = gamepad1.right_stick_x;

        drive.drive(forward, right, rotate);

    }
}
