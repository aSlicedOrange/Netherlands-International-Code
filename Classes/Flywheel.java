package org.firstinspires.ftc.teamcode.Classes;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

/*
Utility Class for Flywheel functionality

To use make an object of the class and pass in hardwareMap, then use the methods below

Example:
private Flywheel flywheel; (Class attribute)

flywheel = new Flywheel(hardwareMap); (Inside Init)

Methods:
public void setRPM(double rpm)
public void setPower(double power)
public double[] getVelocity()
public void setZero()
public void updatePIDF()
*/

public class Flywheel {
    private DcMotorEx flywheelMotorL;
    private DcMotorEx flywheelMotorR;
  
    static final double TICKS_PER_REV_FLYWHEEL = 28;
    static final double GEAR_RATIO_FLYWHEEL = 1;
  
    public static final double flywheelP = 0.1;
    public static final double flywheelI = 0;
    public static final double flywheelD = 0;
    public static final double flywheelF = 32767.0/2800.0;

  public Flywheel(HardwareMap hardwareMap) {
      init(hardwareMap);
  }
  public void init(HardwareMap hardwareMap) {
        flywheelMotorL = hardwareMap.get(DcMotorEx.class, "flywheelMotorLeft");
        flywheelMotorR = hardwareMap.get(DcMotorEx.class, "flywheelMotorRight");

        flywheelMotorL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheelMotorR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        flywheelMotorL.setDirection(DcMotorSimple.Direction.REVERSE);

        flywheelMotorR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        flywheelMotorL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
      
        updatePIDF();
  }

    public void setRPM(double rpm) {
        double ticksPerSecond = (rpm*GEAR_RATIO_FLYWHEEL / 60.0) * TICKS_PER_REV_FLYWHEEL;
        flywheelMotorL.setVelocity(ticksPerSecond);
        flywheelMotorR.setVelocity(ticksPerSecond);
    }
    
    public void setPower(double power) {
        flywheelMotorL.setPower(power);
        flywheelMotorR.setPower(power);
    }

    public void updatePIDF() {
        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(flywheelP, flywheelI, flywheelD, flywheelF); 
        flywheelMotorL.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        flywheelMotorR.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
    }
    
    public double[] getVelocity() {
        double[] velocity = new double[2];
        velocity[0] = flywheelMotorL.getVelocity();
        velocity[1] = flywheelMotorR.getVelocity();

        return (velocity);
    }

    public void setZero() {
        flywheelMotorL.setVelocity(0);
        flywheelMotorR.setVelocity(0);
    }
}
