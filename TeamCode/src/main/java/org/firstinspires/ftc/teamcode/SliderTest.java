package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
//import com.qualcomm.robotcore.hardware.

@TeleOp(name = "Concept: SliderTest", group = "Concept")

public class SliderTest extends LinearOpMode {
    public DcMotorEx slider;

    int maxTicksPosition = 2150;
    int[] levelPositions = {5, 30, 533, 1121, 1678};
    int levelSelected = 0;
    String levelBtnPressed = "";

    public void setSliderLevel(int level)
    {
        slider.setTargetPosition(levelPositions[level + 1]);
        telemetry.addData("Level", level);
        telemetry.update();
        slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slider.setVelocity(2000);
        if (level == -1) {
            slider.setPower(-0.05);
            sleep(300);
            slider.setPower(0);
            slider.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
    }

    public void leave_element_at_level(Button[] buttons)
    {
        if(buttons[0].is_bumped()) {
            levelBtnPressed = "a";
            levelSelected = 1;
        } else if(buttons[1].is_bumped()) {
            levelBtnPressed = "x";
            levelSelected = 2;
        } else if(buttons[2].is_bumped()) {
            levelBtnPressed = "y";
            levelSelected = 3;
        }
        setSliderLevel(levelSelected);
        telemetry.addData("Button pressed", levelBtnPressed);
        telemetry.update();
        sleep(2000);// do servo stuff
        setSliderLevel(0);
    }

    @Override
    public void runOpMode() {
        slider = hardwareMap.get(DcMotorEx.class, "slider");
        slider.setDirection(DcMotorSimple.Direction.REVERSE);
        slider.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        waitForStart();
        Button sl_lvl_1 = new Button();
        Button sl_lvl_2 = new Button();
        Button sl_lvl_3 = new Button();
        Button[] slider_buttons = {sl_lvl_1, sl_lvl_2, sl_lvl_3};
        while (opModeIsActive()) {
            sl_lvl_1.update(gamepad1.x);
            sl_lvl_2.update(gamepad1.y);
            sl_lvl_3.update(gamepad1.a);
            leave_element_at_level(slider_buttons);
            telemetry.addData("Encoder value", slider.getCurrentPosition());
            telemetry.update();
//
        }
    }
}