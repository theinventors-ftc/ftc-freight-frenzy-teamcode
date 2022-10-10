package org.firstinspires.ftc.teamcode;
// import lines were omitted. OnBotJava will add them automatically.
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;


@TeleOp
public class SampleEnc_2 extends LinearOpMode {
    DcMotorEx babis;
    DcMotorEx souias;
    
    @Override
    public void runOpMode() {
        babis = hardwareMap.get(DcMotorEx.class, "babis");
        souias = hardwareMap.get(DcMotorEx.class, "souias");
        
        // Reset the encoder during initialization
        // souias.setDirection(DcMotor.);
        
        babis.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        
        waitForStart();
        
        int targ = 300;
        
        // babis.setTargetPosition(targ);
        
        // Switch to RUN_TO_POSITION mode
        // babis.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        
        // Start the motor moving by setting the max velocity to 200 ticks per second
        babis.setVelocity(200);
 
        // While the Op Mode is running, show the motor's status via telemetry
        while (opModeIsActive()) {
            
            telemetry.addData("position", babis.getCurrentPosition());
            
            telemetry.update();
            
            // if (babis.getCurrentPosition() >= targ) {
            //     babis.setPower(0);
                
            // }
            
            // long error = babis.getCurrentPosition() - targ;
            // babis.
            
            
        }
    }
}