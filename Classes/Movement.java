package org.firstinspires.ftc.teamcode.Classes;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Classes.Odometry;

/*
Utility Class for Coordinate Movement

To use make an object of the class and pass in hardwareMap, then use the methods below

Example:
private Movement movement; (Class attribute)

movement = new Movement(hardwareMap, odo, frontRight, frontLeft, backRight, backLeft); (Inside Init)

Methods:
public boolean moveRobot(double targetX, double targetY, double targetHeading)
public double[][][] chain(double[][] chainNodes, double[][] errors)
*/

public class Movement {
    private DcMotor frontRight;
    private DcMotor frontLeft;
    private DcMotor backRight;
    private DcMotor backLeft;

    private Odometry odo;

    //P Values
    private double minRotateDifference = 0.1;
    private double minForwardDifference = 10.0;
    private double minStrafeDifference = 10.0;
    
    private double divRotateDifference = 90.0;
    private double divForwardDifference = 400.0;
    private double divStrafeDifference = 300.0;

    double currentX;
    double currentY;
    double currentHeading;

public Movement(HardwareMap hardwareMap, Odometry odo) {
  init(hardwareMap, odo);
}

public void init(HardwareMap hardwareMap, Odometry odo) {
  this.odo = odo;
  frontRight = hardwareMap.get(DcMotorEx.class, "frontRight");
  frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeft");
  backRight = hardwareMap.get(DcMotorEx.class, "backRight");
  backLeft = hardwareMap.get(DcMotorEx.class, "backLeft");

  frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
  backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
  frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
  backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        
  frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
  backRight.setDirection(DcMotorSimple.Direction.REVERSE);
  
  currentHeading = -odo.getHeading(AngleUnit.DEGREES);
  currentX = odo.getY();
  currentY = -odo.getX();
  
}

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
        currentHeading = -odo.getHeading(AngleUnit.DEGREES);
        currentX = odo.getY();
        currentY = -odo.getX();
        
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
    


}