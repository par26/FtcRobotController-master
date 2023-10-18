package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.robotcore.hardware.Servo;

// creating this off the basis that 1/2 of claw is controlled by 1 motor, other half another motor
@TeleOp
public class OpenCloseClaw extends OpMode {
    ProgrammingBoard board = new ProgrammingBoard();
    Arm arm = new Arm();

    @Override
    public void init() { //initiating left and right claw
        board.init(hardwareMap);
    }

    @Override
    public void loop() {
        boolean clawPosition = true;

        if (arm.isButtonPressedChecker(gamepad1.a)) { // these if statements closes/opens claw
            clawPosition = !clawPosition;
            if (clawPosition) {
                arm.closeClaw();
            } else {
                arm.openClaw();
            }
        }
    }
}