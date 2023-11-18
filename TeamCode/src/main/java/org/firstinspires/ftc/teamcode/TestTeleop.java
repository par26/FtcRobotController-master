package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp
public class TestTeleop extends OpMode {

    Servo servo;
    Servo servo2;
    //Servo for paper plane launcher
    Servo servo3;

    boolean pa, ca = false;

    boolean pb, cb = false;

    boolean px, cx = false;

    MechanumDrive drive = new MechanumDrive();



    @Override
    public void init() {
        //servo = hardwareMap.get(Servo.class, "testServo1");
        //servo2 = hardwareMap.get(Servo.class, "testServo2");
        servo3 = hardwareMap.get(Servo.class, "testServo3");
        drive.init(hardwareMap);


        //servo.scaleRange(0.75, 0);
        //servo2.scaleRange(0, 0.75);
        //servo3.scaleRange(0,1);  //adjust the position to release the lock

        //motor2.setDirection(Servo.Direction.REVERSE);
        //servo.setPosition(0);
        //servo2.setPosition(0);
        //servo3.setPosition(0);



        servo3.setDirection(Servo.Direction.REVERSE);
        servo3.setPosition(0);

        // testing purpose
        //paper plane starting position

        double position = servo3.getPosition();
        telemetry.addData("position", position);
        telemetry.update();
    }

    @Override
    public void start() {

    }

    @Override
    public void loop() {


        /* pa = ca;
        ca = gamepad1.a;
        if(ca && !pa) {
            servo.setPosition(1);
            servo2.setPosition(1);
        }

        pb = cb;
        cb = gamepad1.b;
        if(cb && !pb) {
            servo.setPosition(0);
            servo2.setPosition(0);
        } */

        px = cx;
        cx = gamepad1.a;
        if(gamepad1.a) {
            servo3.setPosition(1);
            double pos = servo3.getPosition();
            telemetry.addData("position", pos);
            telemetry.update();

        }

        double position = servo3.getPosition();
        telemetry.addData("position", position);
        telemetry.update();


        double forward = drive.squareInput(-gamepad1.left_stick_y * 1.2);
        double rotate = drive.squareInput(gamepad1.left_stick_x * .8);

        double right = -drive.squareInput(gamepad1.right_stick_x);

        drive.drive(forward, right, rotate);


    }

    @Override
    public void stop() {
        //Happens once after stop
        servo3.setPosition(0);
    }




}
