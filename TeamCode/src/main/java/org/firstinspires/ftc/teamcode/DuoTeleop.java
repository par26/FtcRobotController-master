package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;

import java.util.List;

@Config
@com.qualcomm.robotcore.eventloop.opmode.TeleOp
public class DuoTeleop extends OpMode {


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

    public static double leftClose = .20;

    public static double rightClose = .20;

    List<LynxModule> allHubs;
    @Override
    public void init() {

        drive = new SampleMecanumDrive(hardwareMap);


       allHubs = hardwareMap.getAll(LynxModule.class);

        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //rightElbowServo = hardwareMap.get(ServoImplEx.class, "rightElbowServo");
        //leftElbowServo = hardwareMap.get(ServoImplEx.class, "leftElbowServo");

        //rightElbowServo.scaleRange(0, MAX_RETRACT_ANGLE);
        //leftElbowServo.scaleRange(0, MAX_RETRACT_ANGLE);

        leftSlide = hardwareMap.get(DcMotorEx.class, "leftSlide");
        rightSlide = hardwareMap.get(DcMotorEx.class, "rightSlide");

        leftSlide.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        rightSlide.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);


        leftSlide.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rightSlide.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);


        leftSlide.setDirection(DcMotorEx.Direction.REVERSE);

        leftClaw = hardwareMap.get(Servo.class, "leftClaw");

        rightClaw = hardwareMap.get(Servo.class, "rightClaw");

        leftClaw.setDirection(Servo.Direction.REVERSE);

        leftClaw.setPosition(1);
        rightClaw.setPosition(1);

        clawClosed = false;

        leftServo = hardwareMap.get(ServoImplEx.class, "leftServo");
        rightServo = hardwareMap.get(ServoImplEx.class, "rightServo");




        leftServo.setPwmRange(new PwmControl.PwmRange(500, 2500));
        rightServo.setPwmRange(new PwmControl.PwmRange(500, 2500));


        //+rightServo.setDirection(Servo.Direction.REVERSE);
        leftServo.setDirection(Servo.Direction.REVERSE);

        droneServo = hardwareMap.get(Servo.class, "droneServo");

        droneServo.setDirection(Servo.Direction.REVERSE);

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
    }

    @Override
    public void loop() {


        if(Math.abs(leftClaw.getPosition() - leftClose) < .01) {
            clawClosed = true;
        } else {
            clawClosed = false;
        }

        drive.setWeightedDrivePower(
                new Pose2d(
                        -gamepad1.left_stick_y,
                        -gamepad1.left_stick_x,
                        -gamepad1.right_stick_x
                )
        );


        //Open left
        pa = ca;
        ca = gamepad2.a;
        if (ca && !pa) {
            if(clawClosed) {
                leftClaw.setPosition(leftOpen);
                rightClaw.setPosition(rightOpen);
            } else {
                leftClaw.setPosition(leftClose);
                rightClaw.setPosition(rightClose);
                //clawClosed = true;
            }
        }


        pb = cb;
        cb = gamepad2.b;
        if (cb && !pb) {
            if(clawClosed) {
                leftServo.setPosition(0.0);
                rightServo.setPosition(0.0);
            }
        }

        py = cy;
        cy = gamepad2.y;
        if (cy && !py) {
            leftServo.setPosition(1.0);
            rightServo.setPosition(1.0);
        }

        if(gamepad1.dpad_up) {
            droneServo.setPosition(.5);
        }

        double slidePower = Range.clip(gamepad2.right_trigger - gamepad2.left_trigger, -0.75, 0.75);
        if(Math.abs(slidePower) < arm_deadband) {
            leftSlide.setPower(0);
            rightSlide.setPower(0);
        } else {
            leftSlide.setPower(slidePower);
            rightSlide.setPower(slidePower);
        }

        drive.update();
        //powerSlides(slidePower);


    }


    @Override
    public void stop() {
        droneServo.setPosition(0.0);
    }







    public void moveSlidesToPostion(int targetPosition) {
        while(leftSlide.getCurrentPosition() != targetPosition) {
            leftSlide.setTargetPosition(targetPosition);
        }
    }




}
