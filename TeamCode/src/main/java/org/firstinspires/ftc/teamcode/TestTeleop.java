package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp

public class TestTeleop extends OpMode {

    MechanumDrive drive = new MechanumDrive();

    boolean pb, cb = false;

    //rising edge for the a button
    boolean pa, ca = false;

    //rising edge for the x button
    boolean px, cx = false;

    //rising edge for the y button
    boolean py, cy = false;

    //rising edge for left trigger
    boolean plt, clt = false;

    //rising edge for right trigger
    boolean prt, crt = false;

    //rising edge for the left bumper
    boolean plb, clb = false;

    //rising edge for right bumper
    boolean prb, crb = false;

    //rising edge for back button
    boolean pbb, cbb = false;

    //rising edge for start button
    boolean psb, csb = false;

    Servo leftClaw;
    Servo rightClaw;


    @Override
    public void init() {
        leftClaw = hardwareMap.get(Servo.class, "leftClaw");

        rightClaw = hardwareMap.get(Servo.class, "rightClaw");

        leftClaw.scaleRange(0.20, 0.448);
        rightClaw.scaleRange(0.20, 0.448);

        leftClaw.setPosition(1);
        rightClaw.setPosition(0);

    }

    @Override
    public void start() {

    }

    @Override
    public void loop() {

        //Open left
        px = cx;
        cx = gamepad1.x;
        if (cx && !px) {
            leftClaw.setPosition(1);
            double lpos = leftClaw.getPosition();
            telemetry.addData("Position Left", lpos);
            telemetry.update();
            rightClaw.setPosition(0);
            double rpos = rightClaw.getPosition();
            telemetry.addData("Position Right:", rpos);
            telemetry.update();
        }

        //Close left
        pa = ca;
        ca = gamepad1.a;
        if (ca && !pa) {
            leftClaw.setPosition(0);
            double pos = leftClaw.getPosition();
            telemetry.addData("Position Left", pos);
            telemetry.update();
            rightClaw.setPosition(1);
            double rpos = rightClaw.getPosition();
            telemetry.addData("Position right", rpos);
            telemetry.update();
        }

        /*//Open Right
        py = cy;
        cy = gamepad1.y;
        if (cy && !py) {
            rightClaw.setPosition(0);
            double rpos = rightClaw.getPosition();
            telemetry.addData("Position Right:", rpos);
            telemetry.update();

        }
        //Close right
        pb = cb;
        cb = gamepad1.b;
        if (cb && !pb) {
            rightClaw.setPosition(1);
            double rpos = rightClaw.getPosition();
            telemetry.addData("Position right", rpos);
            telemetry.update();
        } */



        /* px = cx;
        cx = gamepad1.a;
        if(gamepad1.a) {
            servo3.setPosition(1);
            double pos = servo3.getPosition();
            telemetry.addData("position", pos);
            telemetry.update();

        }

        double position = servo3.getPosition();
        telemetry.addData("position", position);
        telemetry.update(); */

        double forward = drive.squareInput(-gamepad1.left_stick_y * 1.2);
        double rotate = drive.squareInput(gamepad1.left_stick_x * .8);

        double right = -drive.squareInput(gamepad1.right_stick_x);

        drive.drive(forward, right, rotate);


    }

    public void moveSlowly(double targetPos, double delay) {

    }

    @Override
    public void stop() {
        //Happens once after stop

    }



    //@Override
    //public void stop(); {
    //Happens once after stop

}
