package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.drive.Drive;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.ServoImplEx;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp
public class TestTeleop extends OpMode {
     //
    MechanumDrive drive = new MechanumDrive();

    private final double MAX_RETRACT_ANGLE = 200.0/360.0;



    DcMotorEx leftSlideMotor;

    DcMotorEx rightSlideMotor;


    //ServoImplEx leftElbowServo;
    //ServoImplEx rightElbowServo;

    //DcMotorEx leftSlide;
    final double armManualDeadband = 0.03;

    boolean px = false;
    boolean cx = false;


    boolean py = false;
    boolean cy = false;

    boolean pa = false;
    boolean ca = false;

    boolean pb = false;
    boolean cb = false;

    public final double MAX_POSITION = 2000;
    //Arm arm = new Arm();
    @Override
    public void init() {

        drive.init(hardwareMap);

        //leftSlide = hardwareMap.get(DcMotorEx.class, "leftSlideMotor");
        //leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //leftSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //rightElbowServo = hardwareMap.get(ServoImplEx.class, "rightElbowServo");
        //leftElbowServo = hardwareMap.get(ServoImplEx.class, "leftElbowServo");

        //rightElbowServo.scaleRange(0, MAX_RETRACT_ANGLE);
        //leftElbowServo.scaleRange(0, MAX_RETRACT_ANGLE);


    }

    @Override
    public void loop() {
        double forward = drive.squareInput(gamepad1.left_stick_y * -1);
        double rotate = drive.squareInput(gamepad1.left_stick_x);

        double right = drive.squareInput(gamepad1.right_stick_x);

        drive.drive(forward, right, rotate);


        //powerSlides(slidePower);
    }


}
