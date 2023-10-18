package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp
public class TeleOp extends OpMode {

    //ProgrammingBoard board = new ProgrammingBoard();
    MechanumDrive drive = new MechanumDrive();
    Arm arm = new Arm();

    enum State {
        START,
        WAIT_FOR_SENSOR_RELEASE,
        WAIT_FOR_POT_TURN,
        STOP,
        DONE
    }

    State state;
    @Override
    public void init() {

        drive.init(hardwareMap);
        arm.init(hardwareMap);
        state = State.START;
    }

    @Override
    public void loop() {

        double forward = -gamepad1.left_stick_y;
        double right = gamepad1.left_stick_x;

        double rotate = gamepad1.right_stick_x;

        drive.drive(forward, right, rotate);

        boolean clawPosition = true;

        if (arm.isButtonPressedChecker(gamepad1.a)) { // these if statements closes/opens claw
            state = State.WAIT_FOR_POT_TURN;
            clawPosition = !clawPosition;
            if (clawPosition) {
                arm.closeClaw();
            } else {
                arm.openClaw();
            }}

    }
}
