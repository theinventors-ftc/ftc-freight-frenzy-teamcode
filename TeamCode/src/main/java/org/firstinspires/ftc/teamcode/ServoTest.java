package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Concept: ServoTest", group = "Concept")

public class ServoTest extends LinearOpMode {
    Servo   servo1;
    
    public void runOpMode() {
        servo1 = hardwareMap.get(Servo.class, "servo1");
        
        waitForStart();
        
        while(opModeIsActive()){
            
           
            telemetry.addData(">","servo: " + servo1.getPosition());
            telemetry.update();
        }
    }
}