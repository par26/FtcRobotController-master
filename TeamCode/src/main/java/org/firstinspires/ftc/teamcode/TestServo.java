package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@com.qualcomm.robotcore.eventloop.opmode.TeleOp
public class TestServo extends TeleOp {

    public static double servoAngle;

    public static int servoDirection;
    Servo servo;

    @Override
    public void init() {
        servo = hardwareMap.get(Servo.class, "servo");

        servoAngle = 0;
        //leftServo.setPwmRange();
    }
    @Override
    public void loop() {

        if(servoDirection == 0) {
            servo.setDirection(Servo.Direction.FORWARD);
        } else if(servoDirection == 1) {
            servo.setDirection(Servo.Direction.REVERSE);
        }

        servo.setPosition(servoAngle);
    }
}


