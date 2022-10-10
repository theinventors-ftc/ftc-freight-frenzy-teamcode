package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Concept: MotorBase", group = "Concept")

public class MotorBase extends LinearOpMode {
    private Blinker control_Hub;
    private DcMotor babis;
    private DcMotor souias;

    @Override
    public void runOpMode() {
        babis = hardwareMap.get(DcMotor.class, "babis");
        souias = hardwareMap.get(DcMotor.class, "souias");
        
        waitForStart();
            
        double babisTrigMap = ((gamepad1.right_trigger * 1) / 2) + 1;
        while(opModeIsActive()){
            babis.setPower((gamepad1.right_stick_y / 2)  * babisTrigMap);
            souias.setPower(gamepad1.left_stick_y / 2 * gamepad1.left_trigger / 2);
        }
    }
}