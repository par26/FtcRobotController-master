package org.firstinspires.ftc.teamcode;



import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;


public class Arm {
    double leftOpenPositionValue; //declaring servo's open position value so easily manipulable
    double leftClosePositionValue; //declaring servo's close position value so easily manipulable
    double rightOpenPositionValue; //declaring servo's open position value so easily manipulable
    double rightClosePositionValue; //declaring servo's close position value so easily manipulable

    boolean clawClosed;

    boolean retracted = true;
    Servo leftClawMotor;
    Servo rightClawMotor;

    Servo rotateClaw;


    Servo leftElbowMotor;
    Servo rightElbowMotor;
    DcMotorEx leftSlideMotor;

    DcMotorEx rightSlideMotor;

    //the number of ticks in a rotation of a motor
    public final int SLIDE_MOTOR_TICKS_ROTATION = 123;
    //how many rotations does it take to extend the slides to it's max height
    public final double SLIDE_ROTATIONS_MAX = 8.7;


    public final double MAX_POSITION = SLIDE_ROTATIONS_MAX * SLIDE_MOTOR_TICKS_ROTATION;

    public final double EXTEND_ARM_POS = .85;

    //Claw Length in METERS
    public final double CLAW_LENGTH =.238;

    //The Minimum angle the Elbow can be
    public final double ELBOW_MIN_ANGLE = -13.7;

    //The Mount height of pivoting system in METERS
    public final double PIVOT_HEIGHT = .244;
    public final int armRetractThreshold = 5;

    boolean slideMoving;
    public void init(HardwareMap hwMap) {
        rightClawMotor = hwMap.get(Servo.class, "rightClaw");
        leftClawMotor = hwMap.get(Servo.class, "leftClaw");
        rotateClaw =  hwMap.get(Servo.class, "rotateClaw");

        rightElbowMotor = hwMap.get(Servo.class, "rightElbowMotor");
        leftElbowMotor = hwMap.get(Servo.class, "leftElbowMotor");

        leftSlideMotor = hwMap.get(DcMotorEx.class, "leftSlideMotor");
        leftSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftSlideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        rightSlideMotor = hwMap.get(DcMotorEx.class, "rightSlideMotor");
        rightSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightSlideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        clawClosed = false;
    }

    public boolean isButtonPressedChecker(boolean gamePad) {
        boolean lastPressed = false, isPressed = false; //line 30 - 35 to ensure claw doesn't keep jittering when gamepad is pressed
        boolean clawPosition = true;


        lastPressed = isPressed;
        isPressed = gamePad;
        return lastPressed;
    }
    public void closeClaw() {
        leftClawMotor.setPosition(leftClosePositionValue);
        rightClawMotor.setPosition(rightClosePositionValue);
    }
    public void openClaw() { //ignore this
        leftClawMotor.setPosition(leftOpenPositionValue);
        rightClawMotor.setPosition(rightOpenPositionValue);
    }

    /*
    public void openClaw() {
        double unroundedCurrentLCP = leftClawMotor.getPosition();
        double currentLCP = Math.round(unroundedCurrentLCP * 10) / 10.0;

        while (currentLCP != leftOpenPositionValue) {
            double LCError = leftOpenPositionValue - currentLCP;
            leftClawMotor.setPosition(LCError);
        }

        double unroundedCurrentRCP = rightClawMotor.getPosition();
        double currentRCP = Math.round(unroundedCurrentRCP * 10) / 10.0;

        while (currentLCP != rightOpenPositionValue) {
            double RCError = rightOpenPositionValue - currentRCP;
            leftClawMotor.setPosition(RCError);
        }
    }

    public void closeClaw() {
        double unroundedCurrentLCP = leftClawMotor.getPosition();
        double currentLCP = Math.round(unroundedCurrentLCP * 10) / 10;

        while (currentLCP != leftClosePositionValue) {
            double LCError = leftClosePositionValue - currentLCP;
            leftClawMotor.setPosition(LCError);
        }

        double unroundedCurrentRCP = rightClawMotor.getPosition();
        double currentRCP = Math.round(unroundedCurrentRCP * 10) / 10;

        while (currentLCP != rightClosePositionValue) {
            double RCError = rightClosePositionValue - currentRCP;
            leftClawMotor.setPosition(RCError);
        }
    } */

    //calcs the position the elbow and linear slides have to be in order to reach height
    //height is between the values 0-1 ig. 0 lowest height, 1 max height
    public void clawReachHeight(double height) {
        double targetAngle;
        double targetHeight;


    }

    //moves the claw arm upward
    public void retractArm() {

        //set linear slide motors to x position
        powerSlides(1);

        rotateClaw.setPosition(0);

        leftElbowMotor.setDirection(Servo.Direction.FORWARD);
        rightElbowMotor.setDirection(Servo.Direction.FORWARD);

        leftElbowMotor.setPosition(0);
        rightElbowMotor.setPosition(0);

        if(Math.abs(leftElbowMotor.getPosition()) < .05 && Math.abs(rotateClaw.getPosition()) < 0.1) {
            retracted = true;
        }

    }

    //the moves the claw arm downward to resting position
    public void extendArm() {

        //twist the claw servo by 180 degrees

        leftElbowMotor.setDirection(Servo.Direction.REVERSE);
        rightElbowMotor.setDirection(Servo.Direction.REVERSE);

        //
        leftElbowMotor.setPosition(EXTEND_ARM_POS);
        rightElbowMotor.setPosition(EXTEND_ARM_POS);

        //move rotate claw servo to 180 degress

        rotateClaw.setPosition(0.6);

        if(Math.abs(leftElbowMotor.getPosition() - EXTEND_ARM_POS) < .05 && Math.abs(rotateClaw.getPosition() - 0.6) < 0.1) {
            retracted = false;
        }

    }





    //position true means extended, position false means retracted
    //assumes the motors are connected to two encoder slots, in the scenario, they aren't we may have to adjust
    public void powerSlides(int ticksToBeMoved) {

        slideMoving = true;

        leftSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftSlideMotor.setTargetPosition(ticksToBeMoved);
        rightSlideMotor.setTargetPosition(ticksToBeMoved);
        // set the tolerance of the leftSlide motor
        leftSlideMotor.setTargetPositionTolerance(13);
        rightSlideMotor.setTargetPositionTolerance(13);

        // perform the control loop
        leftSlideMotor.setPower(1);
        rightSlideMotor.setPower(1);
    }

    public void powerSlidesUp() {
        leftSlideMotor.setPower(.7);
        rightSlideMotor.setPower(.7);
    }

    public void powerSlides(double power) {
        leftSlideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightSlideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //use the pid to determien what power to set the slides
        //multiply it by the power variable (-1, 1) as it will the be the velocity varialbe
        while(leftSlideMotor.getCurrentPosition() >= 2 || leftSlideMotor.getCurrentPosition() <= MAX_POSITION) {
            leftSlideMotor.setPower(power);
            rightSlideMotor.setPower(power);
        }

    }

    public void stopSlides() {
        leftSlideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftSlideMotor.setPower(0);
        rightSlideMotor.setPower(0);
    }

    public void powerSlidesDown() {
        leftSlideMotor.setPower(-0.7);
        rightSlideMotor.setPower(-0.7);
    }





}


/*
public class Arm {

    int num;



    public void extendClaw() {


    }

    public void retractClaw() {

    }

    public void openClaw() {

    }

    public void closeClaw() {

    }

    public void bendUp() {

    }

    public void bendDown() {

    }
}*/
