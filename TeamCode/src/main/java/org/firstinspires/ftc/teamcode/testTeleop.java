package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class testTeleop extends OpMode {

    DcMotorEx motor;
    D
    @Override
    public void init() {
        motor = hardwareMap.get(DcMotorEx.class, "tm");

    }

    @Override
    public void loop() {
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        motor.setPower(.75);
    }
}
