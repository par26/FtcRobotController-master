package org.firstinspires.ftc.teamcode;




import com.acmerobotics.roadrunner.control.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.utils.PIDController;

public class Arm {
    double leftOpenPositionValue; //declaring servo's open position value so easily manipulable
    double leftClosePositionValue; //declaring servo's close position value so easily manipulable
    double rightOpenPositionValue; //declaring servo's open position value so easily manipulable
    double rightClosePositionValue; //declaring servo's close position value so easily manipulable

    Servo leftClawMotor;
    Servo rightClawMotor;

    DcMotorEx leftSlideMotor;

    DcMotorEx rightSlideMotor;

    boolean slideMoving;
    public void init(HardwareMap hwMap) {
        rightClawMotor = hwMap.get(Servo.class, "leftClaw");
        leftClawMotor = hwMap.get(Servo.class, "leftClaw");

        leftSlideMotor = hwMap.get(DcMotorEx.class, "leftSlideMotor");
        leftSlideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        rightSlideMotor = hwMap.get(DcMotorEx.class, "rightSlideMotor");
        rightSlideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
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



    //moves the claw arm upward
    public void extendClaw() {

        //set linear slide motors to x position




        //mover the elbow servo to 30 degrees
        powerSlides(1);

    }

    //the moves the claw arm downward to resting position
    public void retractClaw() {
        //move elbow servo to 0 degress

        //set the linear slide motors to reverse direction
        powerSlides(10);

    }

    //position true means extended, position false means retracted
    //assumes the motors are connected to two encoder slots, in the scenario, they aren't we may have to adjust
    public void powerSlides(int ticksToBeMoved) {

        slideMoving = true;

        leftSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);


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
        int pos = leftSlideMotor.getCurrentPosition();
        while(pos >= 10 || pos <= 1000) {
            leftSlideMotor.setPower(power);
            rightSlideMotor.setPower(power);
        }

    }

    public void stopSlides() {
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
