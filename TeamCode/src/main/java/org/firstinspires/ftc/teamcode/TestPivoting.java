package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;

@Config
@com.qualcomm.robotcore.eventloop.opmode.TeleOp
public class TestPivoting extends TeleOp {

    FtcDashboard dashboard;

    public static double leftServoAngle;
    public static double rightServoAngle;

    ServoImplEx leftServo;
    ServoImplEx rightServo;
    @Override
    public void init() {
       leftServo = hardwareMap.get(ServoImplEx.class, "leftServo");
       rightServo = hardwareMap.get(ServoImplEx.class, "rightServo");
       //leftServoAngle = 0;
       //rightServoAngle = 0
        //leftServo.setPwmRange();
        leftServo.setPwmRange(new PwmControl.PwmRange(500, 2500));
        rightServo.setPwmRange(new PwmControl.PwmRange(500, 2500));
    }

    @Override

    public void loop() {
        leftServo.setPosition(leftServoAngle);
        rightServo.setPosition(rightServoAngle);
    }
}
