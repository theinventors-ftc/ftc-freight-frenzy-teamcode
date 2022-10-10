package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.CRServoImpl;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

@TeleOp(name = "Concept: Spanos", group = "Concept")

public class SpanosDemo extends LinearOpMode {
    private DcMotorEx frontLeft;
    private DcMotorEx frontRight;
    private DcMotorEx rearRight;
    private DcMotorEx rearLeft;
    private DcMotor intake;
    private CRServoImpl carousel;
    BNO055IMU imu;

    private boolean gyroFollowEnabled = false;

    private double target = 0;
    private double kp = 0.02;
    boolean do_this_once = true;

    private boolean rBumperPress = false;
    private String dpad_pressed = "";
    private boolean modeBtn_pressed = false;
    private String intakeInDuty = "stop";


        public DcMotorEx slider;

        String levelBtnPressed = "";

    void assignDrivetrainPower(double xG, double yG, double rxG){
        double rx = (-rxG / 2) * (gamepad1.left_trigger + 1);
        double trig = gamepad1.right_trigger;
        double x = -(((xG * 1.1) / 2) * (1 + trig));
        double y = (yG / 2) * (1 + trig);

        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);

        rearRight.setPower((y + x - rx) / denominator);
        rearLeft.setPower((y - x + rx) / denominator);
        frontRight.setPower((y - x - rx) / denominator);
        frontLeft.setPower((y + x + rx) / denominator);
    }

    @Override
    public void runOpMode() {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotorEx.class, "frontRight");
        rearRight = hardwareMap.get(DcMotorEx.class, "rearRight");
        rearLeft = hardwareMap.get(DcMotorEx.class, "rearLeft");
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        carousel = hardwareMap.get(CRServoImpl.class, "carousel");

        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        rearRight.setDirection(DcMotorSimple.Direction.REVERSE);

        slider = hardwareMap.get(DcMotorEx.class, "slider");
        slider.setDirection(DcMotorSimple.Direction.REVERSE);
        slider.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        waitForStart();

        waitForStart();
        double gyroValue, rot, tempMin;
        int tempIndex, i;
        double[] allPower, distanceFromDirections = {0, 0, 0, 0};
        while(opModeIsActive()) {
            gyroValue = - imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;

            if (!gyroFollowEnabled) {
                rot = gamepad1.right_stick_x;
            } else {
                if (do_this_once) {
                    //Define the closest degrees position
                    distanceFromDirections[0] = Math.abs(180 - gyroValue);
                    distanceFromDirections[1] = Math.abs(90 - gyroValue);
                    distanceFromDirections[2] = Math.abs(-90 - gyroValue);
                    distanceFromDirections[3] = Math.abs(-gyroValue);

                    tempMin = distanceFromDirections[0];
                    tempIndex = 0;

                    i = 0;
                    for (double dif : distanceFromDirections) {
                        if (dif < tempMin) {
                            tempMin = dif;
                            tempIndex = i;
                        }
                        i++;
                    }
                    double[] orntsnt = {180, 90, -90, 0};
                    target = orntsnt[tempIndex];
                    do_this_once=false;
                }
                rot = (target - gyroValue) * kp;
            }



//            telemetry.addData("Encoder value", slider.getCurrentPosition());
//            telemetry.update();

            assignDrivetrainPower(gamepad1.left_stick_x, gamepad1.left_stick_y, rot);

            if(gamepad2.dpad_right) {
                carousel.setPower(1);
            } else if(gamepad2.dpad_left) {
                carousel.setPower(-1);
            } else {
                carousel.setPower(0);
            }

            if(gamepad2.right_bumper) {
                slider.setPower(0.4);
            } else if (gamepad2.left_bumper) {
                slider.setPower(-0.4);
            } else {
                slider.setPower(0);
            }



//            telemetry.addData(">", "gyroValue: " + gyroValue + "");
//            telemetry.update();
        }
    }
}