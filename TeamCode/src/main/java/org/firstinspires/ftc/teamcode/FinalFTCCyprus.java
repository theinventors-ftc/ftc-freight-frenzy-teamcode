package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServoImpl;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

@TeleOp(name = "Concept: Final FTC Cyprus", group = "Concept")

public class FinalFTCCyprus extends LinearOpMode {
    private DcMotorEx frontLeft;
    private DcMotorEx frontRight;
    private DcMotorEx rearRight;
    private DcMotorEx rearLeft;
    BNO055IMU imu;

    private boolean gyroFollowEnabled = false;

    private double target = 0;
    private double kp = 0.02;
    boolean do_this_once = true;

    void updateGyroTarget(
            double gyroValue, Button zero, Button plus_90, Button minus_90, Button reverse
    )
    {
        double dist, min_dist;
        int min_dist_idx, max_idx;

        double target_orient = target;
        boolean is_bumped = false;
        if (zero.is_bumped()) {
            target_orient = 0;
            is_bumped = true;
        } else if (plus_90.is_bumped()) {
            target_orient = 90;
            is_bumped = true;
        } else if (minus_90.is_bumped()) {
            target_orient = -90;
            is_bumped = true;
        } else if (reverse.is_bumped()) {
            target_orient = 180;
            is_bumped = true;
        }

        min_dist_idx = 0;
        if (is_bumped) {
            min_dist = Math.abs(target_orient - gyroValue);
            max_idx = (int) Math.ceil(Math.abs(gyroValue) / 360);
            for (int i = -max_idx; i <= max_idx; i++) {
                dist = Math.abs(i * 360 + target_orient - gyroValue);
                if (dist < min_dist) {
                    min_dist_idx = i;
                    min_dist = dist;
                }
            }
        }

        target = min_dist_idx * 360 + target_orient;
    }

    private int find_closest_orient_target(double gyroValue) {
        double dist, min_dist = Math.abs(gyroValue);
        int min_dist_idx = 0;
        int max_idx = (int) Math.ceil(Math.abs(gyroValue) / 90);
        for (int i = -max_idx; i <= max_idx; i++) {
            dist = Math.abs(i * 90 - gyroValue);
            if (dist < min_dist) {
                min_dist_idx = i;
                min_dist = dist;
            }
        }

        return min_dist_idx * 90;
    }
//        //Define the closest degrees position
//        double[] distanceFromOrientation = {
//                Math.abs(180 - gyroValue),
//                Math.abs(90 - gyroValue),
//                Math.abs(-90 - gyroValue),
//                Math.abs(-gyroValue)
//        };
//
//        double tempMin = distanceFromOrientation[0];
//        int tempIndex = 0;
//
//        int i = 0;
//        for (double dif : distanceFromOrientation) {
//            if (dif < tempMin) {
//                tempMin = dif;
//                tempIndex = i;
//            }
//            i++;
//        }
//
//        return tempIndex;
//    }

    void checkModeBtnPressed(Button gyro_follow_toggle){
        if(gyro_follow_toggle.is_bumped()) {
            if(gyroFollowEnabled) {
                gyroFollowEnabled = false;
                do_this_once = true;
            } else {
                gyroFollowEnabled = true;
            }
        }
    }

//    public static final double TICKS_PER_REV = 537.7;

    double [] transformFieldXY2RobotXY(double xG, double yG, double heading) {
        heading *= - Math.PI / 180;
        double [] robotXY = {
                Math.cos(heading) * xG - Math.sin(heading) * yG,
                Math.sin(heading) * xG + Math.cos(heading) * yG
        };

        return robotXY;
    }

    void assignDrivetrainPower(double [] xyG, double rxG){
        double xG = xyG[0];
        double yG = xyG[1];
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
        double gyroValue, rot, cont_gyro_value;

        /**************************** Setup and Initialize IMU (gyro) *****************************/
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        /************************* Setup and Initialize Motors and Servos *************************/
        Servo bucketServo = hardwareMap.get(Servo.class, "bucket");
        DcMotor intake_motor = hardwareMap.get(DcMotorEx.class, "intake");
        DcMotorEx slider_motor = hardwareMap.get(DcMotorEx.class, "slider");
        CRServoImpl carousel_servo = hardwareMap.get(CRServoImpl.class, "carousel");

        slider_motor.setDirection(DcMotorSimple.Direction.REVERSE);
        slider_motor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        // Drivetrain Motors
        frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotorEx.class, "frontRight");
        rearRight = hardwareMap.get(DcMotorEx.class, "rearRight");
        rearLeft = hardwareMap.get(DcMotorEx.class, "rearLeft");

        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        rearRight.setDirection(DcMotorSimple.Direction.REVERSE);

        /************************** Create and Initialize Button Handlers *************************/
        Button intake_proc = new Button();
        Button intake_in = new Button();
        Button intake_out = new Button();
        Button slider_level_1 = new Button();
        Button slider_level_2 = new Button();
        Button slider_level_3 = new Button();
        Button slider_up = new Button();
        Button slider_down = new Button();
        Button carousel_proc = new Button();
        Button carousel_clockwise_turn = new Button();
        Button carousel_counterclockwise_turn = new Button();
        Button gyro_front = new Button();
        Button gyro_right = new Button();
        Button gyro_left = new Button();
        Button gyro_back = new Button();
        Button gyro_follow_toggle = new Button();
        Button bucket_toggle = new Button();

        /************************ Setup and Initialize Mechanisms Objects *************************/
        Intake intake = new Intake(intake_proc, intake_in, intake_out, intake_motor);
        Slider slider = new Slider(
                slider_up, slider_down,
                slider_level_1, slider_level_2, slider_level_3, intake_proc,
                slider_motor
        );
        Carousel carousel = new Carousel(
                carousel_proc, carousel_clockwise_turn, carousel_counterclockwise_turn, carousel_servo
        );
        Bucket bucket = new Bucket(bucket_toggle, bucketServo);

        ContinuousGyro cont_gyro = new ContinuousGyro();

        waitForStart();

        while(opModeIsActive()) {
            /**************** Assign Gamepads Buttons to Button Handlers and Update ***************/
            intake_proc.update(gamepad2.right_bumper);
            intake_in.update(gamepad2.dpad_left);
            intake_out.update(gamepad2.dpad_right);

            slider_up.update(gamepad2.dpad_up);
            slider_down.update(gamepad2.dpad_down);
            slider_level_1.update(gamepad2.b);
            slider_level_2.update(gamepad2.x);
            slider_level_3.update(gamepad2.y);

            gyro_front.update(gamepad1.dpad_up);
            gyro_right.update(gamepad1.dpad_right);
            gyro_left.update(gamepad1.dpad_left);
            gyro_back.update(gamepad1.dpad_down);
            gyro_follow_toggle.update(gamepad1.right_stick_button);

//            carouse_proc.update(gamepad2.b);
            carousel_clockwise_turn.update(gamepad2.left_trigger >= 0.5);
            carousel_counterclockwise_turn.update(gamepad2.right_trigger >= 0.5);
            bucket_toggle.update(gamepad2.a);


            // read gyro values
            gyroValue = - imu.getAngularOrientation(
                    AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES
            ).firstAngle;
            cont_gyro_value = cont_gyro.get_cont_gyro_value(gyroValue);

            updateGyroTarget(cont_gyro_value, gyro_front, gyro_right, gyro_left, gyro_back);
            checkModeBtnPressed(gyro_follow_toggle);

            /****************** Calculate and Assign Drivetrain's Motors' Power *******************/
            if (!gyroFollowEnabled) {
                rot = gamepad1.right_stick_x;
            } else {
                if (do_this_once) {
                    target = find_closest_orient_target(cont_gyro_value);
                    do_this_once = false;
                }
                rot = (target - cont_gyro_value) * kp;
            }
            assignDrivetrainPower(
                    transformFieldXY2RobotXY(
                            gamepad1.left_stick_x, gamepad1.left_stick_y, gyroValue
                    ), rot
            );

            /********************************** Move Mechanisms ***********************************/
            intake.run();
            carousel.run();
            bucket.run();
            slider.run(intake, bucket);

            /************************************* Telemetry **************************************/
            telemetry.addData("Encoder Value:",  slider_motor.getCurrentPosition());
            telemetry.addData("Gyro Value:", gyroValue);
            telemetry.addData("Continuous Gyro Value:", cont_gyro_value);
            telemetry.update();
        }
    }
}