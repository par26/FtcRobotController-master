package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.drive.Drive;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp
public class TestTeleop extends OpMode {


    SampleMecanumDrive drive;



    private final double MAX_RETRACT_ANGLE = 200.0/360.0;



    DcMotorEx leftSlideMotor;

    DcMotorEx rightSlideMotor;


    //ServoImplEx leftElbowServo;
    //ServoImplEx rightElbowServo;

    //DcMotorEx leftSlide;
    final double armManualDeadband = 0.03;

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

    public static double leftOpen = 0.0;

    public static double rightOpen = 0.0;

    public static double leftClose = .2;

    public static double rightClose = .2;
    @Override
    public void init() {

        drive = new SampleMecanumDrive(hardwareMap);

        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //leftSlide = hardwareMap.get(DcMotorEx.class, "leftSlideMotor");
        //leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //leftSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //rightElbowServo = hardwareMap.get(ServoImplEx.class, "rightElbowServo");
        //leftElbowServo = hardwareMap.get(ServoImplEx.class, "leftElbowServo");

        //rightElbowServo.scaleRange(0, MAX_RETRACT_ANGLE);
        //leftElbowServo.scaleRange(0, MAX_RETRACT_ANGLE);


        leftClaw = hardwareMap.get(Servo.class, "leftClaw");

        rightClaw = hardwareMap.get(Servo.class, "rightClaw");

        leftClaw.setDirection(Servo.Direction.REVERSE);

        leftClaw.setPosition(0);
        rightClaw.setPosition(0);
    }

    @Override
    public void loop() {
        drive.setWeightedDrivePower(
                new Pose2d(
                        -gamepad1.left_stick_y,
                        -gamepad1.left_stick_x,
                        -gamepad1.right_stick_x
                )
        );

        drive.update();


        //Open left
        px = cx;
        cx = gamepad1.x;
        if (cx && !px) {
            leftClaw.setPosition(leftOpen);
            double lpos = leftClaw.getPosition();
            telemetry.addData("Position Left", lpos);
            telemetry.update();
            rightClaw.setPosition(rightOpen);
            double rpos = rightClaw.getPosition();
            telemetry.addData("Position Right:", rpos);
            telemetry.update();
        }

        //Close left
        pa = ca;
        ca = gamepad1.a;
        if (ca && !pa) {
            leftClaw.setPosition(leftClose);
            double pos = leftClaw.getPosition();
            telemetry.addData("Position Left", pos);
            telemetry.update();
            rightClaw.setPosition(rightClose);
            double rpos = rightClaw.getPosition();
            telemetry.addData("Position right", rpos);
            telemetry.update();
        }






        //powerSlides(slidePower);
    }


}
