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
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Classes.Flywheel;
import org.firstinspires.ftc.teamcode.Classes.Odometry;

import java.util.ArrayList;


@Autonomous(name = "State Machine Auto")
public class State_Machine_Auto extends OpMode{

    private DcMotorEx frontRight;
    private DcMotorEx frontLeft;
    private DcMotorEx backRight;
    private DcMotorEx backLeft;

    private ElapsedTime stateTimer;

    Flywheel flywheel;
    
    Odometry odo;

    //P Values
    double minDegreeDifference = 0.1;
    double minForwardDifference = 10.0;
    double minStrafeDifference = 10.0;
    
    double divRotateDifference = 30.0;
    double divForwardDifference = 300.0;
    double divStrafeDifference = 300.0;
    
    double targetMoveHeading;
    double targetMoveX;
    double targetMoveY;

    double flywheelTargetRPM = 4000;
    double flywheelLowRPM = 1500;
    
    double[] startPos = {850.9, 3403.6, 0};
    double[] stopPos = new double[3];
    ArrayList<String[]> stateSequence = new ArrayList<>();
    stateSequence.add("MOVE_TO_SHOOT", "MOVE_TO_CHAIN", "MOVE_TO_SHOOT", "MOVE_TO_CHAIN", "MOVE_TO_SHOOT", "MOVE_TO_CHAIN", "MOVE_TO_SHOOT", "PRE_TELEOP")
    boolean hasStopped = false
    
    double[][] chains = {{1000, 1000, 0}, {-1000, -1000, -90}, {2000, 0, 180}, {0, 0, 0}};
    double[][] chainsError = {{250, 250, 0.5}, {250, 250, 0.5}, {15, 15, 0.25}, {10, 10, 0.1}};

    //Enums
    private enum movementState {
        MOVE_TO_SHOOT,
        MOVE_TO_GATE,
        MOVE_TO_BALL1,
        MOVE_TO_BALL2,
        STOP,
        NONE
    }

    private enum turretState {
        FLYWHEEL_ON,
        FLYWHEEL_LOW,
        FLYWHEEL_OFF
    }
    private enum intakeState
        INTAKE_ON,
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
        if (!(Math.abs(errorHeading) < minDegreeDifference)) {
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
        
        flywheel = new Flywheel(hardwareMap);

        odo = new Odometry(hardwareMap, startPos[0], startPos[1], startPos[2]);
        currentX = 0;
        currentY = 0;
        currentHeading = 0;
        
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
        
    }
    
    @Override
    public void start() {
        stateTimer.reset();
    }
    
    @Override
    public void loop(){
        //Testing code
        odo.update();
        currentHeading = -odo.getHeading(AngleUnit.DEGREES);
        currentX = odo.getY();
        currentY = -odo.getX();
        

    private enum movementState {
        MOVE_TO_SHOOT,
        MOVE_TO_GATE,
        MOVE_TO_BALL1,
        MOVE_TO_BALL2,
        STOP,
        NONE
    }

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
                boolean stopCheckCurrent = moveRobot(stopPos[0], stopPos[1], stopPos[2])
                telemetry.addLine("Moving to Position...");
                telemetry.addData("X Data", "Current: %.1f | Target: %.1f | Difference * 100: %.1f", currentX, targetMoveX, 100*(targetMoveX-currentX));
                telemetry.addData("Y Data", "Current: %.1f | Target: %.1f | Difference * 100: %.1f", currentY, targetMoveY, 100*(targetMoveY-currentY));
                telemetry.addData("Heading Data", "Current: %.1f | Target: %.1f | Difference * 100: %.1f", currentHeading, targetMoveHeading, 100*(targetMoveHeading-currentHeading));
                boolean stopCheckPrevious = moveRobot(stopPos[0], stopPos[1], stopPos[2]);
                if (stopCheckCurrent && stopCheckPrevious) {
                    if (!(hasStopped)) {
                        stateTimer.reset();
                        hasStopped = true;
                        intakeS = intakeState.INTAKE_ON
                    }
                    if (stateTimer.milliseconds() > 550) {
                        stateSequence.remove(0);
                    }
                    
                }
                break;
            case MOVE_TO_SHOOT:
                stopPos[0] = 1206.5;
                stopPos[0] = 2159.0;
                stopPos[0] = -40.0;
                movementS = movementState.STOP
            case MOVE_TO_CHAIN:
                if (chains.length == 0) {
                    movementS = movementState.NONE;
                    break;
                }
                chainResult = chain(chains, chainsError);
                if (chainResult[0][0].length > 0) {
                    chains = chainResult[0];
                    chainsError = chainResult[1];
                    telemetry.addLine("Following Chain...");
                    telemetry.addData("X Data", "Current: %.1f | Target: %.1f | Difference * 100: %.1f", currentX, chains[0][0], 100*(chains[0][0]-currentX));
                    telemetry.addData("Y Data", "Current: %.1f | Target: %.1f | Difference * 100: %.1f", currentY, chains[0][1], 100*(chains[0][1]-currentY));
                    telemetry.addData("Heading Data", "Current: %.1f | Target: %.1f | Difference * 100: %.1f", currentHeading, chains[0][2], 100*(chains[0][2]-currentHeading));
                    telemetry.addData("Chains Data", "Length: %d |", chains.length);
                } else {
                    movementS = movementState.NONE;
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
            case INTAKE_OFF:
                intake.setPower(0);
                break;
        }
        telemetry.update();
        
    
    
    
}
}
