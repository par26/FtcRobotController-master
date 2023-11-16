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

    //rising edge for the x button
    boolean px, cx = false;

    //rising edge for the y button
    boolean py, cy = false;

    //rising edge for left trigger
    boolean plt, clt = false;

    //rising edge for right trigger
    boolean prt, crt = false;

    //rising edge for the left bumper
    boolean plb, clb = false;

    //rising edge for right bumper
    boolean prb, crb = false;

    //rising edge for back button
    boolean pbb, cbb = false;

    //rising edge for start button
    boolean psb, csb = false;


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



            switch (state) {
                case FULL_CONTROL_FWD:

                    pb = cb;
                    cb = gamepad1.b;
                    if(cb && !pb) {
                        arm.closeClaw();
                    }
                    plb = clb;
                    clb = gamepad1.left_bumper;

                    prb = crb;
                    crb = gamepad1.right_bumper;

                    if(clb && !prb) {
                        arm.powerSlides(-.7);
                        //adjust the elbow's angle based on the linear slide's position
                    } else if(crb && !prb) {
                        arm.powerSlides(.7);
                    } else {
                        arm.stopSlides();
                    }

                    px = cx;
                    cx = gamepad1.x;

                    if(cx && !px) {
                        arm.retractArm();
                    }

                    if(arm.retracted) {
                        state = State.FULL_CONTROL_BACK;
                    }

                case FULL_CONTROL_BACK:

                    pb = cb;
                    cb = gamepad1.b;

                    if(cb && !pb) {
                        arm.openClaw();
                    }

                    plb = clb;
                    clb = gamepad1.left_bumper;

                    prb = crb;
                    crb = gamepad1.right_bumper;

                    if(clb && !plb) {
                        arm.powerSlides(-.7);
                    } else if(crb && !prb) {
                        arm.powerSlides(.7);
                    } else {
                        arm.stopSlides();
                    }



                    px = cx;
                    cx = gamepad1.x;
                    if(gamepad1.x) {
                        arm.retractArm();
                    }

                    if(arm.retracted == false) {
                        state = State.FULL_CONTROL_FWD;
                    }


            }

        double forward = -gamepad1.left_stick_y;
        double right = gamepad1.left_stick_x;

        double rotate = gamepad1.right_stick_x;

        //drive.driveFieldRelative(forward, right, rotate);

        }
}
