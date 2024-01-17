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
//
////import java.util.List;
//@Config
//@com.qualcomm.robotcore.eventloop.opmode.TeleOp
//public class TeleOp extends OpMode {
//
//    //ProgrammingBoard board = new ProgrammingBoard();
//    SampleMecanumDrive drive;
//    Arm arm = new Arm();
//
//    ElapsedTime timer = new ElapsedTime();
//
//    List<LynxModule> allHubs;
//
//
//    Telemetry telemetry;
//
//
//    ServoImplEx leftServo;
//    ServoImplEx rightServo;
//
//    Servo droneServo;
//
//    //Ports:
//    //leftPivot port 2
//    //right pivot port 5
//    enum State {
//        FULL_CONTROL_FWD,
//
//        RETRACTING,
//
//        FULL_CONTROL_BACK,
//    }
//
//    State state;
//
//    //rising edge for the b button
//    boolean pb, cb = false;
//
//    //rising edge for the a button
//    boolean pa, ca = false;
//
//    //rising edge for the x button
//    boolean px, cx = false;
//
//    //rising edge for the y button
//    boolean py, cy = false;
//
//    //rising edge for left trigger
//    boolean plt, clt = false;
//
//    //rising edge for right trigger
//    boolean prt, crt = false;
//
//    //rising edge for the left bumper
//    boolean plb, clb = false;
//
//    //rising edge for right bumper
//    boolean prb, crb = false;
//
//    //rising edge for back button
//    boolean pbb, cbb = false;
//
//    //rising edge for start button
//    boolean psb, csb = false;
//
//
//
//    static double armManualDeadband = 0.05;
//    //rising edge for the
//
//
//
//
//    /*
//    Control Hub Wiring Specification
//    Port 0 - backRightMotor
//    Port 1 -
//     */
//    @Override
//    public void init() {
//
//        drive = new SampleMecanumDrive(hardwareMap);
//
//        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//
//        arm.init(hardwareMap);
//        state = State.FULL_CONTROL_FWD;
//
//        /*for (LynxModule hub : allHubs) {
//            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
//        }*/
//
//        allHubs = hardwareMap.getAll(LynxModule.class);
//
//        for (LynxModule hub : allHubs) {
//            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
//        }
//
//
//        arm.elbowMoveDown();
//
//        droneServo = hardwareMap.get(Servo.class, "droneServo");
//
//        droneServo.setDirection(Servo.Direction.REVERSE);
//
//        leftServo = hardwareMap.get(ServoImplEx.class, "leftPivot");
//        rightServo = hardwareMap.get(ServoImplEx.class, "rightPivot");
//
//
//        leftServo.setPwmRange(new PwmControl.PwmRange(500, 2500));
//        rightServo.setPwmRange(new PwmControl.PwmRange(500, 2500));
//
//
//        rightServo.setDirection(Servo.Direction.REVERSE);
//
//    }
//
//    @Override
//        public void loop() {
//
//        px = cx;
//        cx = gamepad1.x;
//        if(ca && !pa) {
//            arm.closeClaw();
//        }
//
//
//        py = cy;
//        cy = gamepad1.y;
//        if(cy && !py) {
//            arm.openClaw();
//        }
//
//
//        pb = cb;
//        cb = gamepad1.b;
//
//        if(cb && !pb) {
//            droneServo.setPosition(.2);
//        }
//
//
//        drive.setWeightedDrivePower(
//                new Pose2d(
//                        -gamepad1.left_stick_y,
//                        -gamepad1.left_stick_x,
//                        -gamepad1.right_stick_x
//                )
//        );
//
//        drive.update();
//
//
//        //telemetry.addData("leftSlide Position", arm.leftSlideMotor.getCurrentPosition());
//        double slidePower = (gamepad1.right_trigger -  gamepad1.left_trigger);
//
//        if (Math.abs(slidePower) > armManualDeadband) {
//            arm.powerSlides(slidePower);
//        } else {
//            arm.powerSlides(0.0);
//        }
//
//
//        prb = crb;
//        crb = gamepad1.right_bumper;
//        //EMERGENCY STOP
//        if(gamepad1.right_bumper && !prb) {
//            arm.elbowMoveUp();
//        }
//
//        plb = clb;
//        clb = gamepad1.left_bumper;
//        //EMERGENCY STOP
//        if(gamepad1.left_bumper && !plb) {
//            arm.elbowMoveDown();
//        }
//
//
//
//
//    }
//
//    @Override
//    public void stop() {
//        droneServo.setPosition(0);
//
//        arm.openClaw();
//
//        arm.elbowMoveDown();
//
//        //arm.powerSlides(0);
//
//    }
//}
