package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.CRServo;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import com.qualcomm.hardware.bosch.BNO055IMU;

@TeleOp

public class DriveCorrectionWithGyro extends LinearOpMode {

    BNO055IMU imu;
    Orientation angles;
    private Blinker control_Hub;
    private DcMotor fL;
    private DcMotor fR;
    // private DcMotor rL;
    // private DcMotor rR;
    private CRServo cServo;
    private CRServo cServo_2;
    
    private double target = 0;
    private double error = 0;
    private double kp = 0.02;
    private double speed = 0;
    private double gyroValue = 0;
    private int rotations = 0;
    private double prevGyroValue = 0;
    
    private String pressed = "";
    
    private static double gyroCalc(double value) {
        double fullRotationGyro = value + 180;
        
        
        return value;
    }
    
    @Override
    public void runOpMode() throws InterruptedException {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        fR = hardwareMap.get(DcMotor.class, "FR");
        fL = hardwareMap.get(DcMotor.class, "FL");
        // rL = hardwareMap.get(DcMotor.class, "RL");
        // rR = hardwareMap.get(DcMotor.class, "RR");
        
        fR.setDirection(DcMotor.Direction.REVERSE);
        // rL.setDirection(DcMotor.Direction.REVERSE);
        
        waitForStart();
        while(opModeIsActive()) {
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            
            gyroValue = angles.firstAngle;
            speed = gamepad1.left_stick_y;
            error = (target - gyroCalc(gyroValue)) * kp;

            fL.setPower(speed + error);
            // rL.setPower(speed + error);
            fR.setPower(speed - error);
            // rR.setPower(speed - error);
            
            if (gamepad1.dpad_up){
                pressed = "dpad_up";
            }
            
            if (gamepad1.dpad_down){
                pressed = "dpad_down";
            }
            
            if (gamepad1.dpad_right){
                pressed = "dpad_right";
            }
            
            if (gamepad1.dpad_left){
                pressed = "dpad_left";
            }
            
            if (!gamepad1.dpad_up && pressed == "dpad_up"){
                pressed = "";
                target += 1;
            }
            
            if (!gamepad1.dpad_down && pressed == "dpad_down"){
                pressed = "";
                target -= 1;
            }
            
            if (!gamepad1.dpad_left && pressed == "dpad_left"){
                pressed = "";
                target -= 90;
            }
            
            if (!gamepad1.dpad_right && pressed == "dpad_right"){
                pressed = "";
                target += 90;
            }
            
            
            telemetry.addData(">", "Robot Angle: " + gyroValue);
            telemetry.addData(">", "Robot Target: " + target);
            telemetry.addData(">", "Error: " + ((target + 180) - gyroCalc(gyroValue)));
            // telemetry.addData(">", "Error: " + gyroValue);
            telemetry.update();
        }
    }
}