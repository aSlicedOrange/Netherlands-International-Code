package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;


@TeleOp(name = "Motor Tester")
public class MotorTester extends OpMode{


    private DcMotor motor;



    @Override
    public void init(){

        motor = hardwareMap.get(DcMotor.class, "Motor");
        
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motor.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    @Override
    public void loop(){
    
    if (gamepad1.right_trigger > 0.1) {
        motor.setPower(gamepad1.right_trigger);
    } else if (gamepad1.left_trigger > 0.1) {
        motor.setPower(-gamepad1.left_trigger);
    }
    
    
    
    }
}
