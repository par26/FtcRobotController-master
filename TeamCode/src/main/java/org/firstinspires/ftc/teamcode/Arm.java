package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad2;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Arm {
        boolean gamepadA = gamepad2.a;
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

        public boolean isButtonPressedChecker() {
                boolean lastPressed = false, isPressed = false; //line 30 - 35 to ensure claw doesn't keep jittering when gamepad is pressed
                boolean clawPosition = true;


                lastPressed = isPressed;
                isPressed = gamepadA;
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
}