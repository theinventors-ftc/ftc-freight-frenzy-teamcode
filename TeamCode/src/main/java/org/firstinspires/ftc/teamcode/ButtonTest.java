package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;

@TeleOp(name = "Concept: ButtonTest", group = "Concept")

public class ButtonTest extends LinearOpMode {
    private ColorSensor colorSensor;

    boolean funcBtnPressed = false;
    boolean btnFuncValueReturned = false;
    public boolean btnBumped(boolean btnValue) {
        if(btnValue) funcBtnPressed = true;

        if(funcBtnPressed && !btnValue) {

            funcBtnPressed = false;
            btnFuncValueReturned = true;
        } else {
            btnFuncValueReturned = false;
        }

        return btnFuncValueReturned;
    }
    @Override
    public void runOpMode() {

        waitForStart();

        while(opModeIsActive()) {
            telemetry.addData(">", "Func: " + btnBumped(gamepad1.right_bumper));
            telemetry.update();
        }
    }
}