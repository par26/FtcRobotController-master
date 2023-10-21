package org.firstinspires.ftc.teamcode.utils;


import com.qualcomm.robotcore.util.ElapsedTime;

//implementation of the pid controller
//calcs the amount of power to apply to motor based on constants
public class PIDController {
    double integralSum;
    double Kp;
    double Ki;
    double Kd;
    double reference;
    double lastError;
    ElapsedTime timer;

    public PIDController(double Kp, double Ki, double Kd) {
        timer = new ElapsedTime();
        this.Kp = Kp;
        this.Ki = Ki;
        this.Kd = Kd;
    }


    /**
     * @param reference The motor's target position
     * @param state The motor's current position
     *
     * @return The power to apply to the robot
     */
    public double calcPID(double reference, double state) {
            // calculate the error
            double error = reference - state;

            // rate of change of the error
            double derivative = (error - lastError) / timer.seconds();

            // sum of all error over time
            integralSum = integralSum + (error * timer.seconds());
            //can add optional kf term later for feed forward
            // out += (Kf * reference)
            double out = (Kp * error) + (Ki * integralSum) + (Kd * derivative);

            timer.reset();

            return out;

    }


}
