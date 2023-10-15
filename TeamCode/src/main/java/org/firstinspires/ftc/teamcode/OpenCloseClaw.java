package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.ProgrammingBoard;

// creating this off the basis that 1/2 of claw is controlled by 1 motor, other half another motor
@TeleOp
public class OpenCloseClaw extends OpMode {
    ProgrammingBoard board = new ProgrammingBoard();
    boolean gamepadA = gamepad2.a; // gamepad2 or 1 dunno
    double leftOpenPositionValue; //declaring servo's open position value so easily manipulable
    double leftClosePositionValue; //declaring servo's close position value so easily manipulable
    double rightOpenPositionValue; //declaring servo's open position value so easily manipulable
    double rightClosePositionValue;

    Servo leftClaw; //declaring leftClaw as servo
    Servo rightClaw; //declaring rightClaw as servo


    public void init() { //initiating left and right claw
        board.init(hardwareMap);
        leftClaw = hardwareMap.get(Servo.class, "leftClaw");
        rightClaw = hardwareMap.get(Servo.class, "rightClaw");
    }

    @Override
    public void loop() {
        boolean lastPressed = false, isPressed = false; //line 30 - 35 to ensure claw doesn't keep jittering when gamepad is pressed
        boolean clawPosition = true;


        lastPressed = isPressed;
        isPressed = gamepadA;

        if (isPressed && !lastPressed) { // these if statements closes/opens claw
            clawPosition = !clawPosition;
            if (clawPosition) {
                leftClaw.setPosition(leftClosePositionValue);
                rightClaw.setPosition(rightClosePositionValue);
            } else {
                leftClaw.setPosition(leftOpenPositionValue);
                rightClaw.setPosition(rightOpenPositionValue);
            }
        }
    }
}