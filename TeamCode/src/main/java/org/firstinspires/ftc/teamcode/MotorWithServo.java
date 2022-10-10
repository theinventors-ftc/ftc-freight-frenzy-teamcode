package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Concept: MotorWithServo", group = "Concept")

public class MotorWithServo extends LinearOpMode {
    private Blinker control_Hub;
    private DcMotor babis;
    private DcMotor souias;
    private Servo servo1;

    @Override
    public void runOpMode() {
        babis = hardwareMap.get(DcMotor.class, "babis");
        souias = hardwareMap.get(DcMotor.class, "souias");
        servo1 = hardwareMap.get(Servo.class, "servo1");
        
        waitForStart();
            
        while(opModeIsActive()){
            double babisTrigMap = (gamepad1.right_trigger * 1) + 1;
            babis.setPower((gamepad1.right_stick_y / 2)  * babisTrigMap);
            double souiasTrigMap = (gamepad1.left_trigger * 1) + 1;
            souias.setPower((gamepad1.left_stick_y / 2) * souiasTrigMap);
            telemetry.addData(">", "Babis Trig Map: " + babisTrigMap);
            telemetry.addData(">", "Souias Trig Maps Working Hello This i: " + souiasTrigMap);
            telemetry.update();
            
            if(gamepad1.x) {
                servo1.setPosition(0.5);
            } else {
                servo1.setPosition(0);
            }
            
            if(gamepad1.a) {
                servo1.setPosition(1);
            } else {
                servo1.setPosition(0);
            }
        }
    }
}