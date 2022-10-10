package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Concept: MotorBase_3", group = "Concept")

public class MotorBase_3 extends LinearOpMode {
    private Blinker control_Hub;
    private DcMotor babis;
    private DcMotor souias;

    @Override
    public void runOpMode() {
        babis = hardwareMap.get(DcMotor.class, "babis");
        souias = hardwareMap.get(DcMotor.class, "souias");
        
        waitForStart();
            
        while(opModeIsActive()){
            double babisTrigMap = (gamepad1.right_trigger * 1) + 1;
            babis.setPower((gamepad1.right_stick_y / 2)  * babisTrigMap);
            double souiasTrigMap = (gamepad1.left_trigger * 1) + 1;
            souias.setPower((gamepad1.left_stick_y / 2) * souiasTrigMap);
            telemetry.addData(">", "Babis Trig Map: " + babisTrigMap);
            telemetry.addData(">", "Souias Trig Map: " + souiasTrigMap);
            telemetry.update();
        }
    }
}