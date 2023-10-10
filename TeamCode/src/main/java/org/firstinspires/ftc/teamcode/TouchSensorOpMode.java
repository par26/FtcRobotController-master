package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class TouchSensorOpMode extends OpMode {
    ProgrammingBoard board = new ProgrammingBoard();

    @Override
    public void init() {
        board.init(hardwareMap);
    }

    @Override
    public void loop() {
        telemetry.addData("Touch Sensor State: ",board.getTouchSensorState());
        if(board.isSensorReleased()) {
            telemetry.addLine("Not Pressed");
        } else {
            telemetry.addLine("Pressed");
        }
    }




}
