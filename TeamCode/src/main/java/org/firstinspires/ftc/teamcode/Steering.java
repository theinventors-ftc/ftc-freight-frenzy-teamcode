package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Concept: Steering", group = "Concept")

public class Steering extends LinearOpMode {
    private Blinker control_Hub;
    private DcMotor babis;
    private DcMotor souias;

    @Override
    public void runOpMode() {
        babis = hardwareMap.get(DcMotor.class, "babis");
        souias = hardwareMap.get(DcMotor.class, "souias");
        double speed = 0;
        double babisSpeed = 0;
        double souiasSpeed = 0;
        int accuracy = 2;//1 == least 2 == good 3 accurate
        
        waitForStart();
        
        while(opModeIsActive()){
            
            speed = gamepad1.right_trigger * -1;
            
            babisSpeed = speed + gamepad1.left_stick_x / accuracy;
            souiasSpeed = speed - gamepad1.left_stick_x / accuracy;
            
            babis.setPower(babisSpeed);
            souias.setPower(souiasSpeed);
        }
    }
}