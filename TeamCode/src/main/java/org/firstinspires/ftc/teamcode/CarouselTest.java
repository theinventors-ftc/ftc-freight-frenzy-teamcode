package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServoImpl;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@TeleOp

public class CarouselTest extends LinearOpMode {

    CRServoImpl carouselServo;

    @Override
    public void runOpMode() throws InterruptedException {
        carouselServo = hardwareMap.get(CRServoImpl.class, "carousel");

        waitForStart();
        while(opModeIsActive()) {
            carouselServo.setPower(1);
        }
    }
}