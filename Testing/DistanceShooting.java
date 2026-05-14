package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import org.firstinspires.ftc.teamcode.Classes.Flywheel;


@TeleOp(name = "Motor Tester")
public class MotorTester extends OpMode{

    private DcMotor intake;

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
      
        intake = hardwareMap.get(DcMotor.class, "intakeMotor");
      
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    @Override
    public void loop(){
    
    if (gamepad1.right_trigger > 0.1) {
        flywheel.setRPM(distance(2));
    } else {
        flywheel.setZero();
    }
    if (gamepad1.left_trigger > 0.1) {
        intake.setPower(gamepad1.left_trigger);
    } else {
        intake.setPower(0);
    }

    double curVel = flywheel.getVelocity();
    telemetry.addData("Flywheel Velocity", "%.2f", flywheelVel);
    telemetry.addData("Current Velocity", "%.2f", curVel);
    
    }
}
