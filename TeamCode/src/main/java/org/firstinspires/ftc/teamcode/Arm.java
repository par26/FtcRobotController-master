package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Arm {
        double leftOpenPositionValue = 0.0; //declaring servo's open position value so easily manipulable
        double leftClosePositionValue = 0.0; //declaring servo's close position value so easily manipulable
        double rightOpenPositionValue = 0.0; //declaring servo's open position value so easily manipulable
        double rightClosePositionValue = 0.0; //declaring servo's close position value so easily manipulable

        Servo leftClawServo;
        Servo rightClawServo;

        public void init(HardwareMap hwMap) {
                rightClawServo = hwMap.get(Servo.class, "rightClaw");
                leftClawServo = hwMap.get(Servo.class, "leftClaw");
        }

        public boolean isButtonPressedChecker(boolean gamePad) {
                boolean lastPressed = false, isPressed = false; //line 30 - 35 to ensure claw doesn't keep jittering when gamepad is pressed
                boolean clawPosition = true;


                lastPressed = isPressed;
                isPressed = gamePad;
                return lastPressed;
        }

        public void openClawCL() { //bang bang
                double currentLCPosition = leftClawServo.getPosition();
                double currentRCPosition = rightClawServo.getPosition();

                while (currentLCPosition != leftOpenPositionValue || rightOpenPositionValue != rightOpenPositionValue) {

                        double unroundedLCError = leftOpenPositionValue - currentLCPosition;
                        double LCError = Math.round(unroundedLCError * 10.0) / 10.0;

                        if (LCError > 0) { //will exit when RCError = 0
                                leftClawServo.setPosition(-0.1);
                        } else if (LCError != 0) {
                                leftClawServo.setPosition(+0.1);
                        }


                        double unroundedRCError = rightOpenPositionValue - currentRCPosition;
                        double RCError = Math.round(unroundedRCError * 10.0) / 10.0;


                        if (RCError > 0) { // will exit when RCError = 0
                                rightClawServo.setPosition(-0.1); //questions for big P tomorrow!!!
                        } else if (RCError != 0) {
                                rightClawServo.setPosition(+0.1);
                        }
                }

        }
        /* public void openClawCL() { // proportional
                double currentLCPosition = leftClawServo.getPosition();
                double currentRCPosition = rightClawServo.getPosition();

                while (currentLCPosition != leftOpenPositionValue || rightOpenPositionValue != rightOpenPositionValue) {

                        double unroundedLCError = leftOpenPositionValue - currentLCPosition;
                        double LCError = Math.round(unroundedLCError * 10.0) / 10.0;

                        leftClawServo.setPosition(LCError);

                        double unroundedRCError = rightOpenPositionValue - currentRCPosition;
                        double RCError = Math.round(unroundedRCError * 10.0) / 10.0; */ //doubt this works since not loop :troll:





        public void closeClaw() { // ignore this
                leftClawServo.setPosition(leftClosePositionValue);
                rightClawServo.setPosition(rightClosePositionValue);
        }
        public void openClaw() { // ignore this
                leftClawServo.setPosition(leftOpenPositionValue);
                rightClawServo.setPosition(rightOpenPositionValue);
        }


        //moves the claw arm upward
        public void extendClaw() {
                //set linear slide motors to x position

                //mover the elbow servo to 30 degrees

        }

        //the moves the claw arm downward to resting position
        public void retractClaw() {
                //move elbow servo to 0 degress

                //set the linear slide motors to reverse direction

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
