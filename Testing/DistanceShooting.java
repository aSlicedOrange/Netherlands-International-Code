package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import org.firstinspires.ftc.teamcode.Classes.Flywheel;


@TeleOp(name = "Motor Tester")
public class MotorTester extends OpMode{


    private DcMotor motor1;
    private DcMotor motor2;

    Flywheel flywheel;
    
    private double goalHeight = 1.15;
    private double gravity = 9.816;
    private double Uy = (2*gravity*goalHeight)**0.5;
    private double backplateAngle;
    private double flywheelToBallDiff = 100;
  
    public double distance(double distance) {
        double Ux = (distance*gravity**0.5) / (2*goalHeight)**0.5;
        backplateAngle = Math.atan(Uy / Ux) * (180 / Math.pi);
        double U = (Ux**2 + Uy**2)**0.5;
        double flywheelVel = U * flywheelToBallDiff;

        return(flywheelVel);
    }
  
    @Override
    public void init(){
        flywheel = new Flywheel(hardwareMap);
      
        motor1 = hardwareMap.get(DcMotorEx.class, "Motor1");
        motor2 = hardwareMap.get(DcMotor.class, "Motor2");
      
        motor1.setZeroPowerBehavior(DcMotorEX.ZeroPowerBehavior.FLOAT);
        motor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        
        motor1.setDirection(DcMotorSimple.Direction.REVERSE);
        motor2.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    @Override
    public void loop(){
    
    if (gamepad1.right_trigger > 0.1) {
        motor1.setVelocity(distance(2));
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
