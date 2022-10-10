package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Servo;

public class Bucket {
    private Button toggle;
    private double position;
    private Servo servo;

    public Bucket (Button toggle_bucket, Servo servo)
    {
        this.toggle = toggle_bucket;
        this.servo = servo;
        servo.setPosition(1);
        position = 1;
    }

    public void run()
    {
        if (toggle.is_bumped()) {
            if (position == 0) {
                servo.setPosition(1);
                position = 1;
            } else {
                servo.setPosition(0);
                position = 0;
            }
        }
    }

    public void set_vertical()
    {
        servo.setPosition(0.5);
        position = 0.5;
    }

    public void set_intake()
    {
        servo.setPosition(1);
        position = 1;
    }

    public void set_release()
    {
        servo.setPosition(0);
        position = 0;
    }
}
