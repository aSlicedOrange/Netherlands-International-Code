package org.firstinspires.ftc.teamcode.Testing;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;


@TeleOp(name = "Flywheel PIDF Tuning")
public class Flywheel_PIDF_Tuning extends OpMode{


    private Flywheel flywheel;



    @Override
    public void init(){
        flywheel = new Flywheel(hardwareMap);
    }

    @Override
    public void loop(){
    
    if (gamepad1.right_trigger > 0.1) {
        motor1.setPower(gamepad1.right_trigger);
    } else {
        motor1.setPower(0);
    }
    if (gamepad1.left_trigger > 0.1) {
        motor2.setPower(gamepad1.left_trigger);
    } else {
        motor2.setPower(0);
    }
    
    
    
    }
}
