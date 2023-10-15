package org.firstinspires.ftc.teamcode;

import android.renderscript.ScriptGroup;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class ProgrammingBoard {
    private DigitalChannel touchSensor;
    private DcMotor motor;
    private Servo servo;
    private double ticksPerRotation;


    public void init(HardwareMap hwMap) {
        //get the touchSensor from the control hub
        touchSensor = hwMap.get(DigitalChannel.class, "touch_sensor");
        //
        touchSensor.setMode(DigitalChannel.Mode.INPUT);

        motor = hwMap.get(DcMotor.class, "motor");

        servo = hwMap.get(Servo.class, "servo");

        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        ticksPerRotation = motor.getMotorType().getTicksPerRev();
    }

    public boolean getTouchSensorState() {
        return !touchSensor.getState();
    }

    public void setMotorSpeed(double speed) {
        motor.setPower(speed);

    }

    public boolean isSensorReleased() {
        return touchSensor.getState();
    }

    public double getMotorRotations() {
        return motor.getCurrentPosition() / ticksPerRotation;
    }

    public void brake() {
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void setServoPosition(double position) {
        servo.setPosition(position);
    }


}
