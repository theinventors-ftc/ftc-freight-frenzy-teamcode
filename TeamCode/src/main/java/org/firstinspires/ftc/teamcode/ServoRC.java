 package org.firstinspires.ftc.teamcode;

 import com.qualcomm.robotcore.eventloop.opmode.Disabled;
 import com.qualcomm.robotcore.hardware.CRServo;
 import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
 import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
 import com.qualcomm.robotcore.hardware.CRServoImpl;
 import com.qualcomm.robotcore.hardware.CRServoImplEx;
 import com.qualcomm.robotcore.hardware.Servo;

 @TeleOp(name = "Concept: ServoRC", group = "Concept")

 public class ServoRC extends LinearOpMode {
     Servo servo1;

     @Override
     public void runOpMode() {
         servo1 = hardwareMap.get(Servo.class, "carousel");
        
         waitForStart();

         while(opModeIsActive()){
             servo1.setPosition(gamepad1.x ? 1.0 : 0.0);
             sleep(20);
             idle();
         }
     }
 }
