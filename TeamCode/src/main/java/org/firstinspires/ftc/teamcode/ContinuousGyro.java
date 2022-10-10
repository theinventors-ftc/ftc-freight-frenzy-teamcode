package org.firstinspires.ftc.teamcode;

public class ContinuousGyro
{
    private double previous_gyro_value = 0;
    private double turns = 0;

    public ContinuousGyro() {}

    public double get_cont_gyro_value(double gyro_value)
    {
        if (Math.abs(gyro_value - previous_gyro_value) >= 180)
            if (gyro_value > previous_gyro_value)
                turns -= 1;
            else
                turns += 1;

        previous_gyro_value = gyro_value;

        return gyro_value + 360*turns;
    }
}
