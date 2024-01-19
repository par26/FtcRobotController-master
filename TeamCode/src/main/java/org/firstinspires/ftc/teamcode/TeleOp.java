//package org.firstinspires.ftc.teamcode;
//
//
//
//
//import com.acmerobotics.dashboard.config.Config;
//import com.acmerobotics.roadrunner.geometry.Pose2d;
//import com.qualcomm.hardware.lynx.LynxModule;
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.Gamepad;
//import com.qualcomm.robotcore.hardware.PwmControl;
//import com.qualcomm.robotcore.hardware.Servo;
//import com.qualcomm.robotcore.hardware.ServoController;
//import com.qualcomm.robotcore.hardware.ServoControllerEx;
//import com.qualcomm.robotcore.hardware.ServoImplEx;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//import org.firstinspires.ftc.robotcore.external.Telemetry;
//import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
//
//import java.util.List;

/* @com.qualcomm.robotcore.eventloop.opmode.TeleOp
public class TeleOp extends OpMode {

    //ProgrammingBoard board = new ProgrammingBoard();
    SampleMecanumDrive drive;
    Arm arm = new Arm();

    ElapsedTime timer = new ElapsedTime();

    //List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);




    //Ports:
    //leftPivot port 2
    //right pivot port 5
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


    Telemetry telemetry;
    final double armManualDeadband = 0.03;
    //rising edge for the




    /*
    Control Hub Wiring Specification
    Port 0 - backRightMotor
    Port 1 -
     */
    /*@Override
    public void init() {

        drive = new SampleMecanumDrive(hardwareMap);

        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        arm.init(hardwareMap);
        state = State.FULL_CONTROL_FWD;

        /*for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }*/

    //}

    /*@Override
        public void loop() {

        pa = ca;
        ca = gamepad1.a;
        if(ca && !pa) {
            arm.closeClaw();
        }


        pb = cb;
        cb = gamepad1.b;

        if(cb && !pb) {
            arm.openClaw();
        }


        drive.setWeightedDrivePower(
                new Pose2d(
                        -gamepad1.left_stick_y,
                        -gamepad1.left_stick_x,
                        -gamepad1.right_stick_x
                )
        );

        drive.update();





        //telemetry.addData("leftSlide Position", arm.leftSlideMotor.getCurrentPosition());
        double slidePower = (gamepad1.right_trigger -  gamepad1.left_trigger);


        if (Math.abs(slidePower) > armManualDeadband) {
            arm.powerSlides(slidePower);
        } else {
            arm.powerSlides(0.0);
        }

        px = cx;
        cx = gamepad1.x;

        if(gamepad1.x && !px) {
            arm.retractArm();

        }

        py = cy;
        cy = gamepad1.y;

        if(gamepad1.y && !py) {
            arm.extendArm();

        }

        telemetry.addLine("Position Left"  + arm.leftElbowServo.getPosition());
        telemetry.addLine("Position Right"  + arm.rightElbowServo.getPosition());
        telemetry.update();

        }
} */
