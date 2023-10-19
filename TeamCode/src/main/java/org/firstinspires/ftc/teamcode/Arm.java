package org.firstinspires.ftc.teamcode;




import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Arm {
    double leftOpenPositionValue; //declaring servo's open position value so easily manipulable
    double leftClosePositionValue; //declaring servo's close position value so easily manipulable
    double rightOpenPositionValue; //declaring servo's open position value so easily manipulable
    double rightClosePositionValue; //declaring servo's close position value so easily manipulable

    Servo leftClawMotor;
    Servo rightClawMotor;

    DcMotor leftSlideMotor;

    DcMotor rightSlideMotor;

    public void init(HardwareMap hwMap) {
        rightClawMotor = hwMap.get(Servo.class, "leftClaw");
        leftClawMotor = hwMap.get(Servo.class, "leftClaw");

        leftSlideMotor = hwMap.get(DcMotor.class, "leftSlideMotor");
        leftSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        rightSlideMotor = hwMap.get(DcMotor.class, "rightSlideMotor");
        rightSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public boolean isButtonPressedChecker(boolean gamePad) {
        boolean lastPressed = false, isPressed = false; //line 30 - 35 to ensure claw doesn't keep jittering when gamepad is pressed
        boolean clawPosition = true;


        lastPressed = isPressed;
        isPressed = gamePad;
        return lastPressed;
    }
    /* public void closeClaw() {
        leftClawMotor.setPosition(leftClosePositionValue);
        rightClawMotor.setPosition(rightClosePositionValue);
    }
    public void openClaw() { //ignore this
        leftClawMotor.setPosition(leftOpenPositionValue);
        rightClawMotor.setPosition(rightOpenPositionValue);
    } */

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
    }



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
        powerSlides(-1);

    }

    //position true means extended, position false means retracted
    public void powerSlides(int position) {
        leftSlideMotor.setTargetPosition(position); // the position you want the slides to reach

        leftSlideMotor.setPower(1); // raise at some power

        rightSlideMotor.setTargetPosition(position);
        rightSlideMotor.setPower(1);
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
