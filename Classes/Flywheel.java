package org.firstinspires.ftc.teamcode.Classes;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

public class Flywheel {
    private DcMotorEx flywheelMotorL;
    private DcMotorEx flywheelMotorR;
  
    static final double TICKS_PER_REV_FLYWHEEL = 28;
    static final double GEAR_RATIO_FLYWHEEL = 1;
  
    static final double flywheelP = 0.1;
    static final double flywheelI = 0;
    static final double flywheelD = 0;
    static final double flywheelF = 12.0/6000;

  public Flywheel(HardwareMap hardwareMap) {
      init(hardwareMap);
  }
  public void init(HardwareMap hardwareMap) {
      flywheelMotorL = hardwareMap.get(DcMotorEx.class, "flywheelMotorLeft");
      flywheelMotorR = hardwareMap.get(DcMotorEx.class, "flywheelMotorRight");

      flywheelMotorL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
      flywheelMotorR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
      
      PIDFCoefficients pidfCoefficients = new PIDFCoefficients(flywheelP, 0, 0, flywheelF); 
      flywheelMotorL.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
      flywheelMotorR.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
  }

  public void setFlywheelRPM(double rpm) {
    double ticksPerSecond = (rpm*GEAR_RATIO_FLYWHEEL / 60.0) * TICKS_PER_REV_FLYWHEEL;
    flywheelMotorL.setVelocity(ticksPerSecond);
    flywheelMotorR.setVelocity(ticksPerSecond);
  }

    public void setZero() {
        flywheelMotorL.setVelocity(0);
        flywheelMotorR.setVelocity(0);
    }
}
