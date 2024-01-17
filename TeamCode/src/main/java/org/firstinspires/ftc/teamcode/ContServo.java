//package org.firstinspires.ftc.teamcode;
//
//
//import com.acmerobotics.dashboard.config.Config;
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//import com.qualcomm.robotcore.hardware.CRServo;
//import com.qualcomm.robotcore.hardware.CRServoImplEx;
//import com.qualcomm.robotcore.hardware.PwmControl;
//import com.qualcomm.robotcore.hardware.Servo;
//
//@Config
//@com.qualcomm.robotcore.eventloop.opmode.TeleOp
//public class ContServo extends TeleOp {
//
//    public static double servoAngle;
//
//    public static int servoDirection;
//    CRServoImplEx rightServo;
//
//    CRServoImplEx leftServo;
//
//
//    @Override
//    public void init() {
//        rightServo = hardwareMap.get(CRServoImplEx.class, "rightServo");
//        leftServo = hardwareMap.get(CRServoImplEx.class, "leftServo");
//
//        //servoAngle = 0;
//        //leftServo.setPwmRange();
//
//        rightServo.setPwmRange(new PwmControl.PwmRange(500, 2500));
//        leftServo.setPwmRange(new PwmControl.PwmRange(500, 2500));
//
//
//        rightServo.setDirection(CRServo.Direction.REVERSE);
//
//
//    }
//    @Override
//    public void loop() {
//
//        if (gamepad1.right_bumper) {
//            leftServo.setPwmEnable();
//            rightServo.setPwmEnable();
//            leftServo.setPower(1.0);
//            rightServo.setPower(1.0);
//
//        } else if (gamepad1.left_bumper) {
//            leftServo.setPwmEnable();
//            rightServo.setPwmEnable();
//            leftServo.setPower(-1.0);
//            rightServo.setPower(-1.0);
//        } else {
//            leftServo.setPwmDisable();
//            rightServo.setPwmDisable();
//        }
//
//    }
//}
//
//
//
