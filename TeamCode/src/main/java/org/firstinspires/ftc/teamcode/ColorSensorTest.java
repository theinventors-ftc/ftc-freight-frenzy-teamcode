package org.firstinspires.ftc.teamcode;

//import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
//import com.qualcomm.robotcore.hardware.
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.DcMotor;

//import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@TeleOp(name = "Concept: ColorSensorTest", group = "Concept")

public class ColorSensorTest extends LinearOpMode {
//    private Blinker control_Hub;
//    private DcMotor babis;
//    private DcMotor souias;
    private ColorSensor colorSensor;

    @Override
    public void runOpMode() {
        // Insert whatever initialization your own code does
        colorSensor = hardwareMap.get(ColorSensor.class, "sensor_color");

        waitForStart();

        while(opModeIsActive()) {


            telemetry.addData("Red", colorSensor.red());
            telemetry.addData("Green", colorSensor.green());
            telemetry.addData("Blue", colorSensor.blue());
            telemetry.addData("Alpha", colorSensor.alpha());
            telemetry.addData("Alpha", colorSensor.argb());
            telemetry.update();
            sleep(100);
            // Insert whatever teleop code you're using
        }
    }
}