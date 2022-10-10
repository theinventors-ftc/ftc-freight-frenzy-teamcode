package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServoImpl;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp

public class BucketTest extends LinearOpMode {

    Servo bucketServo;

    @Override
    public void runOpMode() throws InterruptedException {
        bucketServo = hardwareMap.get(Servo.class, "bucket");

        waitForStart();
        while(opModeIsActive()) {
            bucketServo.setPosition(1);
            sleep(500);
            bucketServo.setPosition(0);
            sleep(500);
        }
    }
}