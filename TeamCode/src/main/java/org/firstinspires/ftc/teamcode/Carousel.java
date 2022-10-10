package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServoImpl;

public class Carousel {
    private Button ctrl_proc;
    private Button ctrl_run_forward;
    private Button ctrl_run_reverse;
    private CRServoImpl servo;
    private boolean run_proc = false;

    public Carousel(Button ctrl_proc, Button ctrl_run_forward, Button ctrl_run_reverse, CRServoImpl servo)
    {
        this.ctrl_proc = ctrl_proc;
        this.ctrl_run_forward = ctrl_run_forward;
        this.ctrl_run_reverse = ctrl_run_reverse;
        this.servo = servo;
        this.run_proc = false;
    }

    private void sleep(int ms)
    {
        long start = System.currentTimeMillis();
        while(System.currentTimeMillis() - start < ms) {}
    }

    public void run()
    {
        if (ctrl_proc.is_bumped()) {
            for (int i = 0; i < 10; i++) {
                servo.setPower(1);
                this.sleep(2000);
                servo.setPower(0);
                this.sleep(500);
            }
            run_proc = true;
        } else if (ctrl_run_forward.is_pressed()) {
            servo.setPower(1);
        } else if (! run_proc) {
            servo.setPower(0);
        }
    }
}