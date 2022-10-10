// package org.firstinspires.ftc.teamcode;

// import com.qualcomm.robotcore.hardware.Blinker;
// // import com.qualcomm.robotcore.hardware.DcMotor$RunMode;
// import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
// import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
// import com.qualcomm.robotcore.hardware.DcMotor;
// import com.qualcomm.robotcore.hardware.Gyroscope;


// @TeleOp(name = "Concept: MotorBase_2", group = "Concept")

// public class MotorBase_2 extends LinearOpMode {
//     Blinker control_Hub;
//     DcMotor babis;
//     DcMotor souias;
    
//     @Override
//     public void runOpMode() {

//         // Connect to motor (Assume standard left wheel)
//         // Change the text in quotes to match any motor name on your robot.
//         babis = hardwareMap.get(DcMotor.class, "babis");
//         souias = hardwareMap.get(DcMotor.class, "souias");
        
//         babis.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//         babis.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        
//         babis.setVelocity(200);
        
        
//         waitForStart();

//         while(opModeIsActive()) {
//             telemetry.addData(">", "Position: " + babis.getVelocity());
//         }

//         // Turn off motor and signal done;
//         souias.setPower(0);
//         telemetry.addData(">", "Done");
//         telemetry.update();

//     }
// }