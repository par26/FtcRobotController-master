package org.firstinspires.ftc.teamcode;




import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Arm {
        double leftOpenPositionValue; //declaring servo's open position value so easily manipulable
        double leftClosePositionValue; //declaring servo's close position value so easily manipulable
        double rightOpenPositionValue; //declaring servo's open position value so easily manipulable
        double rightClosePositionValue; //declaring servo's close position value so easily manipulable

        Servo leftClawMotor;
        Servo rightClawMotor;

        public void init(HardwareMap hwMap) {
                rightClawMotor = hwMap.get(Servo.class, "leftClaw");
                leftClawMotor = hwMap.get(Servo.class, "leftClaw");
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
        public void openClaw() {
                leftClawMotor.setPosition(leftOpenPositionValue);
                rightClawMotor.setPosition(rightOpenPositionValue);
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
