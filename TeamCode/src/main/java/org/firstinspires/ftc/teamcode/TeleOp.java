package org.firstinspires.ftc.teamcode;




import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;

import java.util.List;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp
public class TeleOp extends OpMode {

    //ProgrammingBoard board = new ProgrammingBoard();
    MechanumDrive drive = new MechanumDrive();
    Arm arm = new Arm();

    boolean lastBumperMovement, currBumperMovement;

    List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);



    enum State {
        FULL_CONTROL_FWD,

        CLAW_MOVING,

        RETRACTING,

        FULL_CONTROL_BACK,
    }

    State state;
    @Override
    public void init() {

        drive.init(hardwareMap);
        arm.init(hardwareMap);
        state = State.FULL_CONTROL_FWD;

        lastBumperMovement = false;
        currBumperMovement = false;

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
    }

    @Override
    public void loop() {

        double forward = -gamepad1.left_stick_y;
        double right = gamepad1.left_stick_x;

        double rotate = gamepad1.right_stick_x;

        drive.drive(forward, right, rotate);


        lastBumperMovement = currBumperMovement;

        if(gamepad1.left_bumper) {
            arm.powerSlides(-.7);
        } else if(gamepad1.right_bumper) {
            arm.powerSlides(.7);
        } else {
            arm.stopSlides();
        }






    }
}
