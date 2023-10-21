package org.firstinspires.ftc.teamcode;




import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Arm {
        double leftOpenPositionValue; //declaring servo's open position value so easily manipulable
        double leftClosePositionValue; //declaring servo's close position value so easily manipulable
        double rightOpenPositionValue; //declaring servo's open position value so easily manipulable
        double rightClosePositionValue; //declaring servo's close position value so easily manipulable

        Servo leftClawServo;
        Servo rightClawServo;

        DcMotor leftSlideMotor;

        DcMotor rightSlideMotor;

    DcMotor leftSlideMotor;

    DcMotor rightSlideMotor;

    public void init(HardwareMap hwMap) {
        rightClawServo = hwMap.get(Servo.class, "leftClaw");
        leftClawServo = hwMap.get(Servo.class, "leftClaw");

        leftSlideMotor = hwMap.get(DcMotor.class, "leftSlideMotor");
        leftSlideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        rightSlideMotor = hwMap.get(DcMotor.class, "rightSlideMotor");
        rightSlideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

        public boolean isButtonPressedChecker(boolean gamePad) {
                boolean lastPressed = false, isPressed = false; //line 30 - 35 to ensure claw doesn't keep jittering when gamepad is pressed
                boolean clawPosition = true;


                lastPressed = isPressed;
                isPressed = gamePad;
                return lastPressed;
        }


        public void openClaw() {
                double unroundedCurrentLCP = leftClawServo.getPosition();
                double currentLCP = Math.round(unroundedCurrentLCP * 10) / 10.0;

                while (currentLCP != leftOpenPositionValue) {
                    double LCError = leftOpenPositionValue - currentLCP;
                    currentLCP = currentLCP + LCError;
                    leftClawServo.setPosition(currentLCP);
                }

                double unroundedCurrentRCP = rightClawServo.getPosition();
                double currentRCP = Math.round(unroundedCurrentRCP * 10) / 10.0;

                while (currentRCP != rightOpenPositionValue) {
                    double RCError = rightOpenPositionValue - currentRCP;
                    currentRCP = currentRCP + RCError;
                    rightClawServo.setPosition(currentRCP);
                }
        }



        public void closeClaw() {
                double unroundedCurrentLCP = leftClawServo.getPosition();
                double currentLCP = Math.round(unroundedCurrentLCP * 10) / 10;

                while (currentLCP != leftClosePositionValue) {
                        double LCError = leftClosePositionValue - currentLCP;
                        currentLCP = currentLCP + LCError;
                        leftClawServo.setPosition(currentLCP);
                }

                double unroundedCurrentRCP = rightClawServo.getPosition();
                double currentRCP = Math.round(unroundedCurrentRCP * 10) / 10;

                while (currentRCP != rightClosePositionValue) {
                        double RCError = rightClosePositionValue - currentRCP;
                        currentRCP = currentRCP + RCError;
                        leftClawServo.setPosition(currentRCP);
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
