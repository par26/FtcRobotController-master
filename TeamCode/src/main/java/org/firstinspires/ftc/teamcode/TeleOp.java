package org.firstinspires.ftc.teamcode;




import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.hardware.ServoControllerEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.List;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp
public class TeleOp extends OpMode {

    //ProgrammingBoard board = new ProgrammingBoard();
    MechanumDrive drive = new MechanumDrive();
    Arm arm = new Arm();

    ElapsedTime timer = new ElapsedTime();



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


    final double armManualDeadband = 0.03;
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

                    pa = ca;
                    ca = gamepad1.a;
                    if(ca && !pa) {
                        arm.closeClaw();
                    }

                    px = cx;
                    cx = gamepad1.x;

                    if(cx && !px) {
                        arm.extendArm();
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

                    px = cx;
                    cx = gamepad1.x;
                    if(gamepad1.x) {
                        arm.retractArm();
                    }

                    if(!arm.retracted) {
                        state = State.FULL_CONTROL_FWD;
                    }


            }



        double forward = drive.squareInput(-gamepad1.left_stick_y * 1.2);
        double rotate = drive.squareInput(gamepad1.left_stick_x * .8);

        double right = -drive.squareInput(gamepad1.right_stick_x);

        drive.drive(forward, right, rotate);


        double slidePower = gamepad1.right_trigger -  gamepad1.left_trigger;
        //drive.driveFieldRelative(forward, right, rotate);
        if (Math.abs(slidePower) > armManualDeadband) {
            arm.powerSlides(slidePower);
        }

        }
}
