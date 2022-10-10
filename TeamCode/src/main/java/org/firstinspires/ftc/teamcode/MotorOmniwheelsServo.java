package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Concept: MotorOmniwheelsServo", group = "Concept")

public class MotorOmniwheelsServo extends LinearOpMode {
    private Blinker control_Hub;
    private DcMotor FL;
    private DcMotor FR;
    private DcMotor RR;
    private DcMotor RL;
    private CRServo cServo;
    private CRServo cServo_2;

    @Override
    public void runOpMode() {
        FL = hardwareMap.get(DcMotor.class, "FL");
        FR = hardwareMap.get(DcMotor.class, "FR");
        RL = hardwareMap.get(DcMotor.class, "RL");
        RR = hardwareMap.get(DcMotor.class, "RR");
        cServo = hardwareMap.get(CRServo.class, "cServo");
        cServo_2 = hardwareMap.get(CRServo.class, "cServo_2");
        
        FR.setDirection(DcMotor.Direction.REVERSE);
        RL.setDirection(DcMotor.Direction.REVERSE);
        
        
        waitForStart();
            
        while(opModeIsActive()){
            if(gamepad1.a) {
                cServo.setPower(1);
                cServo_2.setPower(1);
            } else {
                cServo.setPower(0);
                cServo_2.setPower(0);
            }
            
            RL.setPower((gamepad1.left_trigger + 1) * (gamepad1.left_stick_y / 2));
            RR.setPower((gamepad1.left_trigger + 1) * (gamepad1.left_stick_y / 2));
            FL.setPower((gamepad1.left_trigger + 1) * (gamepad1.left_stick_y / 2));
            FR.setPower((gamepad1.left_trigger + 1) * (gamepad1.left_stick_y / 2));
            
            RL.setPower((gamepad1.right_trigger + 1) * (gamepad1.right_stick_x / 2) * -1);
            RR.setPower((gamepad1.right_trigger + 1) * (gamepad1.right_stick_x / 2));
            FL.setPower((gamepad1.right_trigger + 1) * (gamepad1.right_stick_x / 2) * -1);
            FR.setPower((gamepad1.right_trigger + 1) * (gamepad1.right_stick_x / 2));
            
            if(gamepad1.x) {
                RL.setPower(0.5);
                RR.setPower(-0.5);
                FL.setPower(-0.5);
                FR.setPower(0.5);
            }
        }
    }
}
