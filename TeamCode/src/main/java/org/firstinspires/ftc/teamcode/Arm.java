package org.firstinspires.ftc.teamcode;




import com.acmerobotics.roadrunner.control.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoControllerEx;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.utils.PIDController;

public class Arm {
    double leftOpenPositionValue; //declaring servo's open position value so easily manipulable
    double leftClosePositionValue; //declaring servo's close position value so easily manipulable
    double rightOpenPositionValue; //declaring servo's open position value so easily manipulable
    double rightClosePositionValue; //declaring servo's close position value so easily manipulable


    boolean clawClosed;

    boolean retracted = true;

    Servo leftClaw;
    Servo rightClaw;

    ServoImplEx leftElbowServo;
    ServoImplEx rightElbowServo;



    DcMotorEx leftSlideMotor;

    DcMotorEx rightSlideMotor;

    //the number of ticks in a rotation of a motor
    public final int SLIDE_MOTOR_TICKS_ROTATION = 384;
    //how many rotations does it take to extend the slides to it's max height
    public final double SLIDE_ROTATIONS_MAX = 8.7 * .52;


    public final double MAX_POSITION = SLIDE_ROTATIONS_MAX * SLIDE_MOTOR_TICKS_ROTATION;

    public final double EXTEND_ARM_POS = .85;


    public static double leftOpen = 0.0;

    public static double rightOpen = 0.0;

    public static double leftClose = .2;

    public static double rightClose = .2;


    boolean slideMoving;
    public void init(HardwareMap hwMap) {

        leftClaw = hwMap.get(Servo.class, "leftServo");

        rightClaw = hwMap.get(Servo.class, "rightServo");

        leftClaw.setDirection(Servo.Direction.REVERSE);

        leftClaw.setPosition(0);
        rightClaw.setPosition(0);

        //leftClawServo.setDirection(Servo.Direction.REVERSE);

       rightElbowServo = hwMap.get(ServoImplEx.class, "rightElbowServo");
        leftElbowServo = hwMap.get(ServoImplEx.class, "leftElbowServo");



        leftSlideMotor = hwMap.get(DcMotorEx.class, "leftSlideMotor");
        leftSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftSlideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        rightSlideMotor = hwMap.get(DcMotorEx.class, "rightSlideMotor");
        rightSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightSlideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);



        clawClosed = false;
    }



    public void closeClaw() {
        leftClaw.setPosition(leftClose);
        rightClaw.setPosition(rightClose);
        clawClosed = true;
    }
    public void openClaw() {//ignore this

        leftClaw.setPosition(leftOpen);
        rightClaw.setPosition(rightOpen);
        clawClosed = false;
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
        //powerSlides(1);



        //leftElbowServo.setDirection(Servo.Direction.FORWARD);
        //rightElbowServo.setDirection(Servo.Direction.FORWARD);

        leftElbowServo.setPosition(0);
        rightElbowServo.setPosition(0);

        if(Math.abs(leftElbowServo.getPosition()) < .05 ) {
            retracted = true;
        }

    }

    //the moves the claw arm downward to resting position
    public void extendArm() {


        //
        leftElbowServo.setPosition(1);
        rightElbowServo.setPosition(1);

        //move rotate claw servo to 180 degress


        if(Math.abs(leftElbowServo.getPosition() - EXTEND_ARM_POS)  > .95) {
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


    public void powerSlides(double power) {
        leftSlideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightSlideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //use the pid to determien what power to set the slides
        //multiply it by the power variable (-1, 1) as it will the be the velocity varialbe
        if(leftSlideMotor.getCurrentPosition() >= 2 || leftSlideMotor.getCurrentPosition() <= (int)MAX_POSITION) {
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

    public void powerSlidesStop() {
        leftSlideMotor.setPower(0);
        rightSlideMotor.setPower(0);
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
