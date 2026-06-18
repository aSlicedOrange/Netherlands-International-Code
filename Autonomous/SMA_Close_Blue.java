package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import java.lang.reflect.Array;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Classes.Flywheel;
import org.firstinspires.ftc.teamcode.Classes.Odometry;
import org.firstinspires.ftc.teamcode.mechanisms.Turret;

import java.util.ArrayList;


@Autonomous(name = "Auto Close Blue")
public class SMA_Close_Blue extends OpMode {

  private DcMotorEx frontRight;
  private DcMotorEx frontLeft;
  private DcMotorEx backRight;
  private DcMotorEx backLeft;

  private ElapsedTime stateTimer;
  private ElapsedTime stopTimer;

  Flywheel flywheel;
  private DcMotor intake;

  private Odometry odo;
  private Turret turret;

  private Servo stopper;

  //P Values
  double minRotateDifference = 0.1;
  double minForwardDifference = 10.0;
  double minStrafeDifference = 10.0;

  double divRotateDifference = 90.0;
  double divForwardDifference = 400.0;
  double divStrafeDifference = 300.0;


  double flywheelTargetRPM = 4000;
  double flywheelLowRPM = 1500;

    double[] startPos = {850.9, 3403.6, 0};
    //double[] startPos = {1206.5, 2159.0, 0};
    //double[] startPos = {1206.5, 3403.6, 0};

    double[] stopPos = new double[3];
    ArrayList<movementState> stateSequence = new ArrayList<>();
    ArrayList<double[][]> chainSequence = new ArrayList<>();
    ArrayList<double[][]> chainErrorSequence = new ArrayList<>();

    boolean hasStopped = false;
    boolean stopCheck = false;
    
    double[][] chains = new double[3][3];
    double[][] chainsError = new double[3][3];
    

    private enum movementState {
        MOVE_TO_SHOOT,
        MOVE_TO_CHAIN,
        STOP,
        PRE_TELEOP,
        NONE
    }
    private enum turretState {
        FLYWHEEL_ON,
        FLYWHEEL_LOW,
        FLYWHEEL_OFF
    }
    private enum intakeState {
        INTAKE_ON,
        INTAKE_LOW,
        INTAKE_OFF
    }
    
    private movementState movementS = movementState.MOVE_TO_SHOOT;
    private turretState turretS = turretState.FLYWHEEL_ON;
    private intakeState intakeS = intakeState.INTAKE_OFF;

    double currentX;
    double currentY;
    double currentHeading;
    

    public double[] getRotatePower(double currentHeading, double targetHeading) {
        double errorHeading = targetHeading - currentHeading;
        
        double divRatio = errorHeading / divRotateDifference;
        double maxPower = 1;
        
        double motorPower = Math.min(Math.abs(divRatio), maxPower) * Math.signum(errorHeading);

        
        double[] rotateMotorPower = {0.0, 0.0, 0.0, 0.0};
        if (!(Math.abs(errorHeading) < minRotateDifference)) {
            rotateMotorPower[0] = motorPower; //fL
            rotateMotorPower[1] = -motorPower; //fR
            rotateMotorPower[2] = motorPower; //bL
            rotateMotorPower[3] = -motorPower; //bR
        }

        return (rotateMotorPower);
    }

    public double[] getForwardPower(double currentY, double targetY) {
        double errorY = targetY - currentY; 
        double divRatio = errorY / divForwardDifference;
        double maxPower = 1;

        double motorPower = Math.min(Math.abs(divRatio), maxPower) * Math.signum(errorY);

        
        double[] forwardMotorPower = {0.0, 0.0, 0.0, 0.0};
        if (Math.abs(errorY) > minForwardDifference) {
            forwardMotorPower[0] = motorPower; //fL
            forwardMotorPower[1] = motorPower; //fR
            forwardMotorPower[2] = motorPower; //bL
            forwardMotorPower[3] = motorPower; //bR
        }

        return (forwardMotorPower);

    }

    public double[] getStrafePower(double currentX, double targetX) {
        double errorX = targetX - currentX;
        
        double divRatio = errorX / divStrafeDifference;
        double maxPower = 1;

        double motorPower = Math.min(Math.abs(divRatio), maxPower) * Math.signum(errorX);

        
        double[] strafeMotorPower = {0.0, 0.0, 0.0, 0.0};
        if (!(Math.abs(errorX) < minStrafeDifference)) {
            strafeMotorPower[0] = motorPower; //fL
            strafeMotorPower[1] = -motorPower; //fR
            strafeMotorPower[2] = -motorPower; //bL
            strafeMotorPower[3] = motorPower; //bR
        }

        return (strafeMotorPower);

    }
    
    public boolean moveRobot(double targetX, double targetY, double targetHeading) {
        double fieldErrorX = targetX - currentX;
        double fieldErrorY = targetY - currentY;
    
        double headingRad = Math.toRadians(currentHeading);
    
        double robotErrorX = fieldErrorX * Math.cos(headingRad) + fieldErrorY * Math.sin(headingRad);
        double robotErrorY = -fieldErrorX * Math.sin(headingRad) + fieldErrorY * Math.cos(headingRad);
    
        //double[] Forward = getForwardPower(robotErrorY);
        //double[] Strafe = getStrafePower(0, robotErrorX);
        //double[] Rotate = getRotatePower(currentHeading, targetHeading);
        
        double[] Forward = getForwardPower(currentY, targetY);
        double[] Strafe = getStrafePower(currentX, targetX);
        double[] Rotate = getRotatePower(currentHeading, targetHeading);

            
        double[] newWheelSpeeds = new double[4];
        
        newWheelSpeeds[0] = Forward[0] + Strafe[0] + Rotate[0];
        newWheelSpeeds[1] = Forward[1] + Strafe[1] + Rotate[1];
        newWheelSpeeds[2] = Forward[2] + Strafe[2] + Rotate[2];
        newWheelSpeeds[3] = Forward[3] + Strafe[3] + Rotate[3];

        double max = Math.max(Math.abs(newWheelSpeeds[0]), Math.abs(newWheelSpeeds[1]));
        max = Math.max(max, Math.abs(newWheelSpeeds[2]));
        max = Math.max(max, Math.abs(newWheelSpeeds[3]));

        if (max > 1.0) {
            newWheelSpeeds[0] /= max;
            newWheelSpeeds[1] /= max;
            newWheelSpeeds[2] /= max;
            newWheelSpeeds[3] /= max;
        }

        
        frontLeft.setPower(newWheelSpeeds[0]);
        frontRight.setPower(newWheelSpeeds[1]);
        backLeft.setPower(newWheelSpeeds[2]);
        backRight.setPower(newWheelSpeeds[3]);

        if (newWheelSpeeds[0] == 0 && newWheelSpeeds[1] == 0 && newWheelSpeeds[2] == 0 && newWheelSpeeds[3] == 0) {
            return true;
        } else {
            return false;
        }
    }
    
    public double[][][] chain(double[][] chainNodes, double[][] errors) {
        double errorX = Math.abs(chainNodes[0][0]) - Math.abs(currentX);
        double errorY = Math.abs(chainNodes[0][1]) - Math.abs(currentY);
        double errorHeading = Math.abs(chainNodes[0][2]) - Math.abs(currentHeading);
        
    double[][] chainNodesNew = new double[chainNodes.length - 1][3];
    double[][] errorsNew = new double[errors.length - 1][3];
        
        
        if (errorX <= errors[0][0] && errorY <= errors[0][1] && errorHeading <= errors[0][2]) {
            for (int i = 1; i < chainNodes.length; i++) {
                chainNodesNew[i - 1][0] = chainNodes[i][0];
                chainNodesNew[i - 1][1] = chainNodes[i][1];
                chainNodesNew[i - 1][2] = chainNodes[i][2];
            }
            chainNodes = chainNodesNew;
            
            for (int i = 1; i < errors.length; i++) {
                errorsNew[i - 1][0] = errors[i][0];
                errorsNew[i - 1][1] = errors[i][1];
                errorsNew[i - 1][2] = errors[i][2];
            }
            errors = errorsNew;
        }
    
        if (chainNodes.length > 0 && errors.length > 0) {
            moveRobot(chainNodes[0][0], chainNodes[0][1], chainNodes[0][2]);
        }
        double[][][] result = {chainNodes, errors};
        return result;
    }
    
    @Override
    public void init(){
        stateTimer = new ElapsedTime();
        stopTimer = new ElapsedTime();
        
        flywheel = new Flywheel(hardwareMap);
        intake = hardwareMap.get(DcMotor.class, "intakeMotor");

        odo = new Odometry(hardwareMap);
        odo.setStartingPosition(-startPos[1], startPos[0], -startPos[2]);
        currentX = 0;
        currentY = 0;
        currentHeading = 0;
        turret = new Turret(hardwareMap, telemetry);
        
        stopper = hardwareMap.get(Servo.class, "stopper");
        
        frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeft");
        backLeft = hardwareMap.get(DcMotorEx.class, "backLeft");
        frontRight = hardwareMap.get(DcMotorEx.class, "frontRight");
        backRight = hardwareMap.get(DcMotorEx.class, "backRight");

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);
        
        stateSequence.add(movementState.MOVE_TO_SHOOT);
        stateSequence.add(movementState.STOP);
        stateSequence.add(movementState.MOVE_TO_CHAIN);
        stateSequence.add(movementState.MOVE_TO_SHOOT);
        stateSequence.add(movementState.STOP);
        stateSequence.add(movementState.MOVE_TO_CHAIN);
        stateSequence.add(movementState.MOVE_TO_SHOOT);
        stateSequence.add(movementState.STOP);
        stateSequence.add(movementState.MOVE_TO_CHAIN);
        stateSequence.add(movementState.PRE_TELEOP);
        stateSequence.add(movementState.STOP);
        stateSequence.add(movementState.NONE);
        chainSequence.add(new double[][] {{936, 1919.7, -90}, {484.2, 1941.2, -90}, {406.4, 1866.9, -90}});
        chainSequence.add(new double[][] {{999.5, 1360.9, -90}, {215.9, 1397, -90}, {698.5, 1447.8, -70}});
        chainSequence.add(new double[][] {{330.2, 1440.2, -50}, {215.9, 1440.2, -50}, {869.9, 1936.7, -60}});
        chainErrorSequence.add(new double[][] {{100, 250, 0.5}, {50, 100, 0.5}, {10, 10, 0.1}});
        chainErrorSequence.add(new double[][] {{100, 250, 0.5}, {10, 10, 0.1}, {250, 250, 10}});
        chainErrorSequence.add(new double[][] {{10, 10, 0.1}, {10, 10, 0.1}, {400, 400, 10}});
        chains = chainSequence.get(0);
        chainsError = chainErrorSequence.get(0);
    }

    @Override
    public void start() {
        stateTimer.reset();
        stopTimer.reset();
    }
    
    @Override
    public void loop() {
        //Testing code
        turret.update();
        odo.update();
        currentHeading = -odo.getHeading(AngleUnit.DEGREES);
        currentX = odo.getY();
        currentY = -odo.getX();
        


        double[][][] chainResult;

        switch (movementS) {
            case NONE:
                frontLeft.setPower(0);
                frontRight.setPower(0);
                backLeft.setPower(0);
                backRight.setPower(0);
                turretS = turretState.FLYWHEEL_OFF;
                intakeS = intakeState.INTAKE_OFF;
                break;
      case STOP:
          boolean stopCheck = moveRobot(stopPos[0], stopPos[1], stopPos[2]);
          telemetry.addLine("Moving to Position...");
          telemetry.addData("X Data", "Current: %.1f | Target: %.1f | Difference * 100: %.1f", currentX, stopPos[0], 100*(stopPos[0]-currentX));
          telemetry.addData("Y Data", "Current: %.1f | Target: %.1f | Difference * 100: %.1f", currentY, stopPos[1], 100*(stopPos[1]-currentY));
          telemetry.addData("Heading Data", "Current: %.1f | Target: %.1f | Difference * 100: %.1f", currentHeading, stopPos[2], 100*(stopPos[2]-currentHeading));
          double[] flywheelVel = flywheel.getVelocity();
          double flywheelSpeed = (flywheelVel[0]+flywheelVel[1])/2.0;
                if (!stopCheck) {
                    stopTimer.reset();
                }
                if (stopTimer.milliseconds() > 50) {
                    if (flywheelSpeed > 1500){
                        if (stopper.getPosition() > 0.3) {
                            stopper.setPosition(stopper.getPosition()-0.05);
                        }
                    if (!(hasStopped && stopper.getPosition() > 0.5)) {
                        stateTimer.reset();
                        hasStopped = true;
                        intakeS = intakeState.INTAKE_ON;
                    } else {
                        intakeS = intakeState.INTAKE_OFF;
                    }
                    if (stateTimer.milliseconds() > 1000) {
                        turretS = turretState.FLYWHEEL_LOW;
                        intakeS = intakeState.INTAKE_OFF;
                        stopper.setPosition(0);
                        stateSequence.remove(0);
                    }
                    }
          }
          break;
      case MOVE_TO_SHOOT:
          stopPos[0] = 1206.5;
          stopPos[1] = 2159.0;
          stopPos[2] = -40.0;
          stateSequence.remove(0);
          turretS = turretState.FLYWHEEL_ON;
          stopCheck = false;
          break;
      case PRE_TELEOP:
          stopPos[0] = 1506.5;
          stopPos[1] = 2459.0;
          stopPos[2] = -50.0;
          stateSequence.remove(0);
          turretS = turretState.FLYWHEEL_ON;
          stopCheck = false;
          break;
      case MOVE_TO_CHAIN:
          if (chains.length == 0) {
              chainSequence.remove(0);
              chainErrorSequence.remove(0);
              stateSequence.remove(0);
              break;
          }
          chainResult = chain(chains, chainsError);
          if (chainResult[0].length > 0) {
              chains = chainResult[0];
              chainsError = chainResult[1];
              telemetry.addLine("Following Chain...");
              telemetry.addData("X Data", "Current: %.1f | Target: %.1f | Difference * 100: %.1f", currentX, chains[0][0], 100*(chains[0][0]-currentX));
              telemetry.addData("Y Data", "Current: %.1f | Target: %.1f | Difference * 100: %.1f", currentY, chains[0][1], 100*(chains[0][1]-currentY));
              telemetry.addData("Heading Data", "Current: %.1f | Target: %.1f | Difference * 100: %.1f", currentHeading, chains[0][2], 100*(chains[0][2]-currentHeading));
              telemetry.addData("Chains Data", "Length: %d |", chains.length);
          } else {
              chainSequence.remove(0);
              chainErrorSequence.remove(0);
              stateSequence.remove(0);
          }
          break;
  }

  switch (turretS) {
      case FLYWHEEL_ON:
          //Turret backplate and rev code here
          flywheel.setRPM(flywheelTargetRPM);
          break;
      case FLYWHEEL_LOW:
          flywheel.setRPM(flywheelLowRPM);
          break;
      case FLYWHEEL_OFF:
          flywheel.setZero();
          break;
  }
  
  switch (intakeS) {
      case INTAKE_ON:
          intake.setPower(1);
          break;
      case INTAKE_LOW:
          intake.setPower(0.5);
          break;
      case INTAKE_OFF:
          intake.setPower(0);
          break;
  }
  telemetry.addData("stateSequence: ", stateSequence.get(0));
  telemetry.addData("movementS: ", movementS);
  telemetry.addData("turretS: ", turretS);
  telemetry.addData("intakeS: ", intakeS);
  telemetry.update();


  chains = chainSequence.get(0);
  chainsError = chainErrorSequence.get(0);
  
  switch (stateSequence.get(0)) {
      case MOVE_TO_SHOOT:
          movementS = movementState.MOVE_TO_SHOOT;
          break;
      case MOVE_TO_CHAIN:
          movementS = movementState.MOVE_TO_CHAIN;
          intakeS = intakeState.INTAKE_LOW;
          break;
      case STOP:
          movementS = movementState.STOP;
          stopCheck = false;
          break;
      case PRE_TELEOP:
          movementS = movementState.PRE_TELEOP;
          stopCheck = false;
          break;
      case NONE:
          movementS = movementState.NONE;
          break;
          
  }
}
}

