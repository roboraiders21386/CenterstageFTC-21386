package org.firstinspires.ftc.teamcode.OpModes.teleOP;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.hardware.Servo;

public class ServoTester {
    private Servo servo1;
    private Servo servo2;

    public void runOpMode()
    {
        servo1  = hardwareMap.get(Servo.class, "Servo1");
        servo2 = hardwareMap.get(Servo.class, "Servo2");

        if(gamepad1.right_bumper)
        {
            //1 clockwise 2 anticlockwise
           // servo1.Power(1);
            //servo2.Power(-1);
        }
        if(gamepad1.left_bumper)
        {
            //other way around!
            //servo1.Power(-1);
            //servo2.Power(1);
        }
    }
}
