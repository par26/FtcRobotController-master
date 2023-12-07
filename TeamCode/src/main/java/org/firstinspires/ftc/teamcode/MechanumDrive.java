package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class MechanumDrive {
    private DcMotorEx frontRightMotor;
    private DcMotorEx frontLeftMotor;
    private DcMotorEx backRightMotor;
    private DcMotorEx backLeftMotor;

    private IMU imu;

    public void init(HardwareMap hardwareMap) {
        frontLeftMotor = hardwareMap.get(DcMotorEx.class, "front_left_motor");
        frontRightMotor = hardwareMap.get(DcMotorEx.class, "front_right_motor");
        backLeftMotor = hardwareMap.get(DcMotorEx.class, "back_left_motor");
        backRightMotor = hardwareMap.get(DcMotorEx.class, "back_right_motor");
        imu = hardwareMap.get(IMU.class, "imu");

        RevHubOrientationOnRobot revHubOrientationOnRobot = new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.RIGHT, RevHubOrientationOnRobot.UsbFacingDirection.UP);

        imu.initialize(new IMU.Parameters(revHubOrientationOnRobot));


        frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        frontRightMotor.setDirection(DcMotor.Direction.REVERSE);
        backRightMotor.setDirection(DcMotor.Direction.REVERSE);


        frontLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    private void setPowers(double frontRightPower, double frontLeftPower, double backLeftPower, double backRightPower) {
        frontLeftMotor.setPower(frontLeftPower);
        frontRightMotor.setPower(frontRightPower);
        backLeftMotor.setPower(backLeftPower);
        backRightMotor.setPower(backRightPower*.75);
    }

    public double squareInput(double input) {
        double result = input * input;

        if(input < 0) {
            result *= -1;
        }

        return result;
    }

    public void drive(double forward, double right, double rotate) {

        //double denominator = Math.max(Math.abs(right) + Math.abs(forward) + Math.abs(rotate), 1);

        double fLeftPow = Range.clip(forward + rotate + right, -0.85, .85);
        double bLeftPow = Range.clip(forward + rotate - right, -0.85, .85);
        double fRightPow = Range.clip(forward - rotate - right, -0.85, .85);
        double bRightPow = Range.clip(forward - rotate + right, -0.85, .85);


        setPowers(fLeftPow, fRightPow, bLeftPow, bRightPow);
    }

    public void driveFieldRelative(double forward, double right, double rotate) {
        double robotAngle = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        //convert to polar
        double theta = Math.atan2(forward, right);

        double r = Math.hypot(forward, right);

        theta = AngleUnit.normalizeRadians(theta - robotAngle);

        double newForward = r * Math.sin(theta);
        double newRight = r * Math.cos(theta);

        this.drive(newForward, newRight, rotate);
    }

    //add function to set robot at specific angle
    public void rotate(double angle) {
        //take the current angle through imu

        //use pid controller to adjust it's rotation
    }


}
