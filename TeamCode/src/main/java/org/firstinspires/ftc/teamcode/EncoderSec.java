// package org.firstinspires.ftc.teamcode;

// import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
// import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
// import com.qualcomm.robotcore.hardware.Blinker;
// import com.qualcomm.robotcore.hardware.DcMotor;
// import com.qualcomm.robotcore.hardware.Gyroscope;

// @TeleOp(name = "Concept: Encoder", group = "Concept")

// public class EncoderSec extends LinearOpMode {
//     private Blinker control_Hub;
//     private DcMotor babis;
//     private Gyroscope imu;

//     @Override
//     public void runOpMode() {

//         // Connect to motor (Assume standard left wheel)
//         // Change the text in quotes to match any motor name on your robot.
//         babis = hardwareMap.get(DcMotor.class, "babis");
//         // souias = hardwareMap.get(DcMotor.class, "souias");
        
//         motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        
//         babis.setVelocity(200);
        
//         waitForStart();

//         while(opModeIsActive()) {
//             telemetry.addData(">", "Velocity: " + babis.getVelocity());
//         }

//         // Turn off motor and signal done;
//         babis.setPower(0);
//         telemetry.addData(">", "Done");
//         telemetry.update();

//     }
// }