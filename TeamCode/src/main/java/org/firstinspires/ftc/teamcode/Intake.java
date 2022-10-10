package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class Intake {
    private Button ctrl_proc;
    private Button ctrl_in;
    private Button ctrl_out;
    private DcMotor motor;
    private boolean waiting = false;
    private String state = "stop";
    private AsynchronousSleep sleeper = new AsynchronousSleep();
    private int time;
    public Intake(Button ctrl_proc, Button ctrl_in, Button ctrl_out, DcMotor motor)
    {
        this.ctrl_proc = ctrl_proc;
        this.ctrl_in = ctrl_in;
        this.ctrl_out = ctrl_out;
        this.motor = motor;
    }

    public boolean is_busy()
    {
        return state != "stop";
    }

    public boolean is_running()
    {
        return state == "running";
    }

    public boolean is_reverse()
    {
        return state == "reverse";
    }

    public void run()
    {
        if(ctrl_proc.is_bumped() || waiting) {
            sleeper.update();
            if (state == "running") {
                state = "reverse";
                motor.setPower(-1);
                sleeper.wait(500);
                waiting = true;
            } else {
                if (sleeper.is_ready()) {
                    motor.setPower(0);
                    state = "stop";
                    sleeper.reset();
                    waiting = false;
                } else if (!sleeper.is_busy()) {
                    state = "running";
                    motor.setPower(1);
                }
            }
        } else if((state == "stop") && ctrl_in.is_pressed()) {
            motor.setPower(1);
        } else if ((state == "stop") && ctrl_out.is_pressed()) {
            motor.setPower(-1);
        } else if (state == "stop") {
            motor.setPower(0);
        }
    }
}
