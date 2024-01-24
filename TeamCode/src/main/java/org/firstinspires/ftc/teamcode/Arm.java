//package org.firstinspires.ftc.teamcode;
//
//
//
//
//import com.acmerobotics.roadrunner.control.PIDFController;
//import com.qualcomm.robotcore.hardware.CRServoImplEx;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorEx;
//import com.qualcomm.robotcore.hardware.DcMotorSimple;
//import com.qualcomm.robotcore.hardware.HardwareMap;
//import com.qualcomm.robotcore.hardware.Servo;
//import com.qualcomm.robotcore.hardware.ServoControllerEx;
//import com.qualcomm.robotcore.hardware.ServoImplEx;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//import org.firstinspires.ftc.teamcode.utils.PIDController;
//
//public class Arm {
//    double leftOpenPositionValue; //declaring servo's open position value so easily manipulable
//    double leftClosePositionValue; //declaring servo's close position value so easily manipulable
//    double rightOpenPositionValue; //declaring servo's open position value so easily manipulable
//    double rightClosePositionValue; //declaring servo's close position value so easily manipulable
//
//
//    boolean clawClosed;
//
//    boolean retracted = true;
//
//    Servo leftClaw;
//    Servo rightClaw;
//
//    public ServoImplEx leftElbowServo;
//    public ServoImplEx rightElbowServo;
//
//
//
//    DcMotorEx leftSlideMotor;
//
//    DcMotorEx rightSlideMotor;
//
//    //the number of ticks in a rotation of a motor
//    public final int SLIDE_MOTOR_TICKS_ROTATION = 384;
//    //how many rotations does it take to extend the slides to it's max height
//    public final double SLIDE_ROTATIONS_MAX = 8.7 * 0.9;
//
//
//    public final double MAX_POSITION = SLIDE_ROTATIONS_MAX * SLIDE_MOTOR_TICKS_ROTATION;
//
//    public final double EXTEND_ARM_POS = .6666;
//
//
//    public static double leftOpen = 0.0;
//
//    public static double rightOpen = 0.0;
//
//    public static double leftClose = .2;
//
//    public static double rightClose = .2;
//
//
//    boolean slideMoving;
//    public void init(HardwareMap hwMap) {
//
//        leftClaw = hwMap.get(Servo.class, "leftClaw");
//
//        rightClaw = hwMap.get(Servo.class, "rightClaw");
//
//        leftClaw.setDirection(Servo.Direction.REVERSE);
//
//        leftClaw.setPosition(0);
//        rightClaw.setPosition(0);
//
//        //leftClawServo.setDirection(Servo.Direction.REVERSE);
//
//       rightElbowServo = hwMap.get(ServoImplEx.class, "rightPivot");
//       leftElbowServo = hwMap.get(ServoImplEx.class, "leftPivot");
//
//
//        rightElbowServo.setDirection(ServoImplEx.Direction.REVERSE);
//
//
//        leftSlideMotor = hwMap.get(DcMotorEx.class, "leftSlide");
//        //leftSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        leftSlideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        leftSlideMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
//
//        rightSlideMotor = hwMap.get(DcMotorEx.class, "rightSlide");
//        //rightSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        rightSlideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        rightSlideMotor.setDirection(DcMotorEx.Direction.REVERSE);
//        rightSlideMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
//
//
//
//        clawClosed = false;
//    }
//
//
//
//
//    public void closeClaw() {
//        leftClaw.setPosition(leftClose);
//        rightClaw.setPosition(rightClose);
//        clawClosed = true;
//    }
//    public void openClaw() {//ignore this
//
//        leftClaw.setPosition(leftOpen);
//        rightClaw.setPosition(rightOpen);
//        clawClosed = false;
//    }
//
//
//
//    //calcs the position the elbow and linear slides have to be in order to reach height
//    //height is between the values 0-1 ig. 0 lowest height, 1 max height
//    public void clawReachHeight(double height) {
//        double targetAngle;
//        double targetHeight;
//
//
//    }
//
//    //moves the claw arm upward
//    public void elbowMoveUp() {
//        leftElbowServo.setPosition(1.0);
//        rightElbowServo.setPosition(1.0);
//    }
//
//    public void elbowMoveDown() {
//        leftElbowServo.setPosition(0.0);
//        rightElbowServo.setPosition(0.0);
//    }
//
//
//
//
//
//    //position true means extended, position false means retracted
//    //assumes the motors are connected to two encoder slots, in the scenario, they aren't we may have to adjust
//
//
//
//
//    public void powerSlides(double power) {
//
//        if(leftSlideMotor.getPosition() >= 2 && leftSlideMotor.getPosition() <= 1800) {
//
//        leftSlideMotor.setPower(power);
//        rightSlideMotor.setPower(power);
//
//      }
//    }
//
//    public void stopSlides() {
//        //leftSlideMotor.s
//    }
//
//
//
//
//
//
//}
//
//
///*
//public class Arm {
//
//    int num;
//
//
//
//    public void extendClaw() {
//
//
//    }
//
//    public void retractClaw() {
//
//    }
//
//    public void openClaw() {
//
//    }
//
//    public void closeClaw() {
//
//    }
//
//    public void bendUp() {
//
//    }
//
//    public void bendDown() {
//
//    }
//}*/








