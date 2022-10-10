package org.firstinspires.ftc.teamcode;

import static java.lang.Thread.sleep;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

public class Slider
{
    private Button ctrl_up;
    private Button ctrl_down;
    private Button level_1_ctrl;
    private Button level_2_ctrl;
    private Button level_3_ctrl;
    private Button intake_ctrl;
    private DcMotorEx motor;
    private String state = "park";
    private String intake_state = "stop";
    private Servo bucket;

//    private int maxTicksPosition = 2150;
    private int[] levelPositions = {0, 200, 533, 1121, 1678};
    private int levelSelected = 0;

    private boolean run_to_level = false;

    public Slider(
            Button ctrl_up, Button ctrl_down,
            Button level_1_ctrl, Button level_2_ctrl, Button level_3_ctrl, Button intake_ctrl,
            DcMotorEx motor
    )
    {
        this.ctrl_up = ctrl_up;
        this.ctrl_down = ctrl_down;
        this.level_1_ctrl = level_1_ctrl;
        this.level_2_ctrl = level_2_ctrl;
        this.level_3_ctrl = level_3_ctrl;
        this.intake_ctrl = intake_ctrl;
        this.motor = motor;
    }

    public void setSliderLevel(int level)
    {
        motor.setTargetPosition(levelPositions[level + 1]);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setVelocity(1600);
    }

//    private void sleep(int ms)
//    {
//        long start = System.currentTimeMillis();
//        while(System.currentTimeMillis() - start < ms) {}
//    }

    public void run(Intake intake, Bucket bucket)
    {
        if (level_1_ctrl.is_bumped()) {
            levelSelected = 1;
            run_to_level = true;
        } else if (level_2_ctrl.is_bumped()) {
            levelSelected = 2;
            run_to_level = true;
        } else if (level_3_ctrl.is_bumped()) {
            levelSelected = 3;
            run_to_level = true;
        } else if (intake_ctrl.is_bumped()) {
            levelSelected = -1;
            run_to_level = true;
        }
        if (run_to_level) {
            if (levelSelected == -1) {
                if (intake.is_running()) {
                    bucket.set_intake();
                    setSliderLevel(-1);
                } else {
                    setSliderLevel(0);
                    bucket.set_vertical();
                    if (!motor.isBusy())
                        run_to_level = false;
                }
            } else {
                if (state == "park") {
                    setSliderLevel(levelSelected);
                    state = "in_position";
                } else if (state == "in_position" && (
                        (level_1_ctrl.is_bumped() && levelSelected==1) ||
                        (level_2_ctrl.is_bumped() && levelSelected==2) ||
                        (level_3_ctrl.is_bumped() && levelSelected==3)
                    )
                ) {
                    setSliderLevel(0);
                    if (!motor.isBusy()) {
                        state = "park";
                        run_to_level = false;
                    }
                }
            }
        } else if (ctrl_up.is_pressed()) {
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motor.setPower(0.5);
        } else if (ctrl_down.is_pressed()) {
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motor.setPower(-0.5);
        } else {
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motor.setPower(0);
        }
    }
}