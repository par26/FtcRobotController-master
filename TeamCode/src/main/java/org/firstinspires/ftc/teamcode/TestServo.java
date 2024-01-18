//package org.firstinspires.ftc.teamcode;
//
//import com.acmerobotics.dashboard.config.Config;
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//import com.qualcomm.robotcore.hardware.CRServoImplEx;
//import com.qualcomm.robotcore.hardware.Servo;
//
//@Config
//@com.qualcomm.robotcore.eventloop.opmode.TeleOp
//public class TestServo extends OpMode {
//
//    public static double servoAngle;
//
//    public static int servoDirection;
//    Servo servo;
//
//
//    @Override
//    public void init() {
//        servo = hardwareMap.get(Servo.class, "servo");
//
//        //servoAngle = 0;
//        //leftServo.setPwmRange();
//
//        servo.setDirection(Servo.Direction.REVERSE);
//
//
//    }
//    @Override
//    public void loop() {
//        px = cx;
//        cx = gamepad1.x;
//        if (cx &) {
//            servo.setPosition(0);
//
//        }
//        //Close left
//        pa = ca;
//        ca = gamepad1.a;
//        if (ca && !pa) {
//            servo.setPosition(0.3);
//        }
//    }
//}
//
//
