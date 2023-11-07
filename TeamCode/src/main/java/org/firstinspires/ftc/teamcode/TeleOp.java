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



    List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);



    enum State {
        FULL_CONTROL_FWD,

        RETRACTING,

        FULL_CONTROL_BACK,
    }

    State state;

    //rising edge for the b button
    boolean pb, cb = false;
    //rising edge for the a button
    boolean pa, ca = false;
    //previous left trigger
    boolean plt, clt = false;
    //rising edge for the

    @Override
    public void init() {

        drive.init(hardwareMap);
        arm.init(hardwareMap);
        state = State.FULL_CONTROL_FWD;

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

            switch (state) {
                case FULL_CONTROL_FWD:

                    boolean lastClawMovement = false, currClawMovement = false;
                    if(gamepad1.b) {
                        arm.closeClaw();
                    }

                    if(gamepad1.left_bumper) {
                        arm.powerSlides(-.7);
                        //adjust the elbow's angle based on the linear slide's position
                    } else if(gamepad1.right_bumper) {
                        arm.powerSlides(.7);
                    } else {
                        arm.stopSlides();
                    }


                    if(gamepad1.x) {
                        arm.retractArm();
                    }

                    if(arm.retracted) {
                        state = State.FULL_CONTROL_BACK;
                    }

                case FULL_CONTROL_BACK:

                    if(gamepad1.b) {
                        arm.openClaw();
                    }

                    if(gamepad1.left_bumper) {
                        arm.powerSlides(-.7);
                    } else if(gamepad1.right_bumper) {
                        arm.powerSlides(.7);
                    } else {
                        arm.stopSlides();
                    }

                    if(gamepad1.left_bumper) {
                        arm.powerSlides(-.7);
                    } else if(gamepad1.right_bumper) {
                        arm.powerSlides(.7);
                    } else {
                        arm.stopSlides();
                    }

                    if(gamepad1.x) {
                        arm.retractArm();
                    }

                    if(arm.retracted == false) {
                        state = State.FULL_CONTROL_FWD;
                    }


            }



        }
}
