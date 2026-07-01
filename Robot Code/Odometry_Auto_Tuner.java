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
import org.firstinspires.ftc.teamcode.Classes.Flywheel;
import org.firstinspires.ftc.teamcode.Classes.Odometry;

import java.util.ArrayList;


@Autonomous(name = "Odometry Auto Tuner")
public class Odometry_Auto_Tuner extends OpMode{

    private DcMotorEx frontRight;
    private DcMotorEx frontLeft;
    private DcMotorEx backRight;
    private DcMotorEx backLeft;


    Flywheel flywheel;
    
    Odometry odo;

    //P Values
    double minDegreeDifference = 0.1;
    double minForwardDifference = 10.0;
    double minStrafeDifference = 10.0;
    
    double divRotateDifference = 90.0;
    double divForwardDifference = 400.0;
    double divStrafeDifference = 300.0;
    
    double targetMoveHeading;
    double targetMoveX;
    double targetMoveY;
    
    double[][] chains = {{1000, 1000, 0}, {-1000, -1000, -90}, {2000, 0, 180}, {0, 0, 0}};
    double[][] chainsError = {{250, 250, 0.5}, {250, 250, 0.5}, {15, 15, 0.25}, {10, 10, 0.1}};
    
    
ArrayList<double[]> chainsModular = new ArrayList<>();
ArrayList<double[]> chainsErrorModular = new ArrayList<>();
    int currentErrorModularState = 0;
    double[][] ErrorModularStates = {{10, 10, 0.1}, {250, 250, 0.25}, {500, 500, 0.5}};

    //Enums
    private enum movementState {
        MOVE_TO_SHOOT,
        MOVE_TO_GATE,
        MOVE_TO_BALL1,
        MOVE_TO_BALL2
    }

    private enum miscState {
        FLYWHEEL_ON,
        FLYWHEEL_OFF,
        INTAKE_ON,
        INTAKE_OFF
    }
    
    private enum controlState {
        MOVE_TO_POS,
        MOVE_TO_CHAIN,
        MOVE_TO_CHAIN_MODULAR,
        NONE
    }
    
    private controlState currentState = controlState.NONE;

    double currentX;
    double currentY;
    double currentHeading;
    
    private com.qualcomm.robotcore.hardware.Gamepad currentGamepad1 = new com.qualcomm.robotcore.hardware.Gamepad();
    private com.qualcomm.robotcore.hardware.Gamepad previousGamepad1 = new com.qualcomm.robotcore.hardware.Gamepad();

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

        flywheel = new Flywheel(hardwareMap);

        odo = new Odometry(hardwareMap);
        odo.setStartingPosition(0, 0, 0);
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
    public void loop(){
        //Testing code
        odo.update();
        currentHeading = -odo.getHeading(AngleUnit.DEGREES);
        currentX = odo.getY();
        currentY = -odo.getX();
        

        if (gamepad1.a) {
            currentState = controlState.MOVE_TO_POS;
        } else if (gamepad1.b) {
            currentState = controlState.MOVE_TO_CHAIN;
        } else if (gamepad1.y) {
            currentState = controlState.NONE;
        } else if (gamepad1.x) {
            currentState = controlState.MOVE_TO_CHAIN_MODULAR;
        }
        
        previousGamepad1.copy(currentGamepad1);
        currentGamepad1.copy(gamepad1);
        
        if (currentGamepad1.dpad_up && !previousGamepad1.dpad_up && chainsModular.size()<5) {
            chainsModular.add(new double[]{currentX, currentY, currentHeading});
            chainsErrorModular.add(ErrorModularStates[currentErrorModularState]);
            
        } else if (currentGamepad1.dpad_down && !previousGamepad1.dpad_down && chainsModular.size()>0) {
            chainsModular.remove(chainsModular.size() - 1);
            chainsErrorModular.remove(chainsErrorModular.size() - 1);
            
        } else if (currentGamepad1.dpad_left && !previousGamepad1.dpad_left && currentErrorModularState>0) {
            currentErrorModularState--;
            
        } else if (currentGamepad1.dpad_right && !previousGamepad1.dpad_right && currentErrorModularState<3) {
            currentErrorModularState++;
            
        } else if (currentGamepad1.right_bumper && !previousGamepad1.right_bumper) {
            StringBuilder chainBuilder = new StringBuilder("AUTO CHAIN POINTS: {");
            for (int i = 0; i < chainsModular.size(); i++) {
                double[] node = chainsModular.get(i);
                chainBuilder.append(String.format("{%.0f, %.0f, %.0f}", node[0], node[1], node[2]));
                if (i < chainsModular.size() - 1) {
                    chainBuilder.append(", ");
                }
            }
            chainBuilder.append("}");
            
            StringBuilder errorBuilder = new StringBuilder("AUTO CHAIN ERRORS: {");
            for (int i = 0; i < chainsErrorModular.size(); i++) {
                double[] err = chainsErrorModular.get(i);
                errorBuilder.append(String.format("{%.0f, %.0f, %.2f}", err[0], err[1], err[2]));
                if (i < chainsErrorModular.size() - 1) {
                    errorBuilder.append(", ");
                }
            }
            errorBuilder.append("}");
            
            com.qualcomm.robotcore.util.RobotLog.dd("FTC_AUTO", "CHAIN POINTS: " +chainBuilder.toString());
            com.qualcomm.robotcore.util.RobotLog.dd("FTC_AUTO", "CHAIN ERRORS: " +errorBuilder.toString());
        }
        
        
        double[][][] chainResult;
        switch (currentState) {
            case NONE:
                frontLeft.setPower(0);
                frontRight.setPower(0);
                backLeft.setPower(0);
                backRight.setPower(0);
                
                StringBuilder chainBuilder = new StringBuilder("CHAIN POINTS: {");
                for (int i = 0; i < chainsModular.size(); i++) {
                    double[] node = chainsModular.get(i);
                    chainBuilder.append(String.format("{%.0f, %.0f, %.0f}", node[0], node[1], node[2]));
                    if (i < chainsModular.size() - 1) {
                        chainBuilder.append(", ");
                    }
                }
                chainBuilder.append("}");
                
                StringBuilder errorBuilder = new StringBuilder("CHAIN ERRORS: {");
                for (int i = 0; i < chainsErrorModular.size(); i++) {
                    double[] err = chainsErrorModular.get(i);
                    errorBuilder.append(String.format("{%.0f, %.0f, %.2f}", err[0], err[1], err[2]));
                    if (i < chainsErrorModular.size() - 1) {
                        errorBuilder.append(", ");
                    }
                }
            errorBuilder.append("}");
    
    
    
    
                telemetry.addLine("--- RECORDED CHAIN DATA ---");
                telemetry.addData("POINTS:", chainBuilder.toString());
                telemetry.addData("ERRORS:", errorBuilder.toString());
                telemetry.addLine("--------------------------");
                
                com.qualcomm.robotcore.util.RobotLog.d(chainBuilder.toString());
                com.qualcomm.robotcore.util.RobotLog.d(errorBuilder.toString());
                break;
            case MOVE_TO_POS:
                if (!(moveRobot(0, 0, 0))) {
                    telemetry.addLine("Moving to Position...");
                    telemetry.addData("X Data", "Current: %.1f | Target: %.1f | Difference * 100: %.1f", currentX, targetMoveX, 100*(targetMoveX-currentX));
                    telemetry.addData("Y Data", "Current: %.1f | Target: %.1f | Difference * 100: %.1f", currentY, targetMoveY, 100*(targetMoveY-currentY));
                    telemetry.addData("Heading Data", "Current: %.1f | Target: %.1f | Difference * 100: %.1f", currentHeading, targetMoveHeading, 100*(targetMoveHeading-currentHeading));
                } else {
                    currentState = controlState.NONE;
                }
                break;
            case MOVE_TO_CHAIN:
                if (chains.length == 0) {
                    currentState = controlState.NONE;
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
                    currentState = controlState.NONE;
                }
                break;
            case MOVE_TO_CHAIN_MODULAR:
                if (chainsModular.isEmpty()) {
                    currentState = controlState.NONE;
                    break;
                } 
                double[][] modularArray = chainsModular.toArray(new double[0][]);
                double[][] modularErrorArray = chainsErrorModular.toArray(new double[0][]);
                chainResult = chain(modularArray, modularErrorArray);
                chainsModular = new ArrayList<>(java.util.Arrays.asList(chainResult[0]));
                chainsErrorModular = new ArrayList<>(java.util.Arrays.asList(chainResult[1]));
                if (chainsModular.size() > 0) {
                    telemetry.addLine("Following Modular Chain...");
                    telemetry.addData("X Data", "Current: %.1f | Target: %.1f | Difference * 100: %.1f", currentX, chainsModular.get(0)[0], 100*(chainsModular.get(0)[0]-currentX));
                    telemetry.addData("Y Data", "Current: %.1f | Target: %.1f | Difference * 100: %.1f", currentY, chainsModular.get(0)[1], 100*(chainsModular.get(0)[1]-currentY));
                    telemetry.addData("Heading Data", "Current: %.1f | Target: %.1f | Difference * 100: %.1f", currentHeading, chainsModular.get(0)[2], 100*(chainsModular.get(0)[2]-currentHeading));
                } else {
                    currentState = controlState.NONE;
                }
                break;
        }
        
        telemetry.addData("Chains Modular Data", "Length: %d |", chainsModular.size());
        telemetry.addData("Chains Modular Error Data", "State: %d |", currentErrorModularState);
        telemetry.update();
        
    
    
    
}
}