package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.drive.Drive;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;

@Config
@com.qualcomm.robotcore.eventloop.opmode.TeleOp
public class TestTeleop extends OpMode {


    SampleMecanumDrive drive;



    private final double MAX_RETRACT_ANGLE = 200.0/360.0;

    boolean clawClosed;
    boolean elbowExtended;

    public static double idle_power = 0.0;

    ServoImplEx leftServo;
    ServoImplEx rightServo;
    DcMotorEx leftSlideMotor;

    DcMotorEx rightSlideMotor;


    //ServoImplEx leftElbowServo;
    //ServoImplEx rightElbowServo;

    //DcMotorEx leftSlide;
    final double arm_deadband = 0.05;

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

    DcMotorEx leftSlide;
    DcMotorEx rightSlide;
    Servo droneServo;

    public static double leftOpen = 0.0;

    public static double rightOpen = 0.0;

    public static double leftClose = .2;

    public static double rightClose = .2;
    @Override
    public void init() {

        drive = new SampleMecanumDrive(hardwareMap);

        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //rightElbowServo = hardwareMap.get(ServoImplEx.class, "rightElbowServo");
        //leftElbowServo = hardwareMap.get(ServoImplEx.class, "leftElbowServo");

        //rightElbowServo.scaleRange(0, MAX_RETRACT_ANGLE);
        //leftElbowServo.scaleRange(0, MAX_RETRACT_ANGLE);

        leftSlide = hardwareMap.get(DcMotorEx.class, "leftSlide");
        rightSlide = hardwareMap.get(DcMotorEx.class, "rightSlide");

        leftSlide.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rightSlide.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);


        leftSlide.setDirection(DcMotorEx.Direction.REVERSE);

        leftClaw = hardwareMap.get(Servo.class, "leftClaw");

        rightClaw = hardwareMap.get(Servo.class, "rightClaw");

        leftClaw.setDirection(Servo.Direction.REVERSE);

        leftClaw.setPosition(0);
        rightClaw.setPosition(0);

        leftServo = hardwareMap.get(ServoImplEx.class, "leftServo");
        rightServo = hardwareMap.get(ServoImplEx.class, "rightServo");



        leftServo.setPwmRange(new PwmControl.PwmRange(500, 2500));
        rightServo.setPwmRange(new PwmControl.PwmRange(500, 2500));


        droneServo = hardwareMap.get(Servo.class, "droneServo");

        droneServo.setDirection(Servo.Direction.REVERSE);

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


        double slidePower = Range.clip(gamepad1.right_trigger - gamepad1.left_trigger, -0.75, 0.75);


        if(Math.abs(slidePower) < arm_deadband) {
            leftSlide.setPower(slidePower);
            rightSlide.setPower(slidePower);
        } else {
            leftSlide.setPower(idle_power);
            rightSlide.setPower(idle_power);
        }
        //Open left
        py = cy;
        cy = gamepad1.y;
        if (cy && !py) {
            leftClaw.setPosition(leftOpen);
            rightClaw.setPosition(rightOpen);
        }

        //Close left
        pa = ca;
        ca = gamepad1.a;
        if (ca && !pa) {
            leftClaw.setPosition(leftClose);
            rightClaw.setPosition(rightClose);

        }


        px = cx;
        cx = gamepad1.x;
        if (clb && !plb) {
            leftServo.setPosition(0);
            rightServo.setPosition(0);
        }

        pb = cb;
        cb = gamepad1.b;
        if (cb && !pb) {
            leftServo.setPosition(1.0);
            rightServo.setPosition(1.0);
        }




        if(gamepad1.dpad_up) {
            droneServo.setPosition(.5);
        }


        drive.update();
        //powerSlides(slidePower);
    }


    @Override
    public void stop() {
        droneServo.setPosition(0.0);
    }

}
