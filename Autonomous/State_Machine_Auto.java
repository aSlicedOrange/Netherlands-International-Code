package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import org.firstinspires.ftc.teamcode.classes.Flywheel;

@Autonomous(name = "SM_Auto")
public class SM_Auto extends OpMode{

    private DcMotorEx frontRight;
    private DcMotorEx frontLeft;
    private DcMotorEx backRight;
    private DcMotorEx backLeft;

    private DcMotorEx flywheelMotorL;
    private DcMotorEx flywheelMotorR;

    GoBildaPinpointDriver odo;


    private enum state {
        MOVE_TO_SHOOT,
        MOVE_TO_GATE,
        MOVE_TO_BALL1,
        MOVE_TO_BALL2
    }

    //For Testing
    private double deltaHeading = 0;
    private double deltaX = 0;
    private double deltaY = 0;
    

    public double[] getRotatePower(double currentHeading, double targetHeading) {
        double minDegreeDifference = 0.1;
        double errorHeading = targetHeading - currentHeading;

        
        double divRotateDifference = 10.0;
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

        return (rotateMotorPower)
    }

    public double[] getForwardPower(double currentY, double targetY) {
        double minYDifference = 0.5;
        double errorY = targetY - currentY;

        double divYDifference = 150.0;
        double divRatio = errorY / divYDifference;
        double maxPower = 1;

        double motorPower = Math.min(Math.abs(divRatio), maxPower) * Math.signum(errorY);

        
        double[] forwardMotorPower = {0.0, 0.0, 0.0, 0.0};
        if (!(Math.abs(errorY) < minYDifference)) {
            forwardMotorPower[0] = motorPower; //fL
            forwardMotorPower[1] = motorPower; //fR
            forwardMotorPower[2] = motorPower; //bL
            forwardMotorPower[3] = motorPower; //bR
        }

        return (forwardMotorPower)

    }


    public double[] getStrafePower(double currentX, double targetX) {
        double minXDifference = 0.5;
        double errorX = targetX - currentX;

        double divXDifference = 150.0;
        double divRatio = errorX / divXDifference;
        double maxPower = 1;

        double motorPower = Math.min(Math.abs(divRatio), maxPower) * Math.signum(errorX);

        
        double[] strafeMotorPower = {0.0, 0.0, 0.0, 0.0};
        if (!(Math.abs(errorX) < minXDifference)) {
            strafeMotorPower[0] = motorPower; //fL
            strafeMotorPower[1] = -motorPower; //fR
            strafeMotorPower[2] = -motorPower; //bL
            strafeMotorPower[3] = motorPower; //bR
        }

        return (strafeMotorPower)

    }
    
    public boolean moveRobot(double targetX, double targetY, double targetHeading) {
        pose2D pos = odo.getPosition();
        double currentHeading = pos.getHeading(AngleUnit.DEGREES);
        double currentX = pos.getX(DistanceUnit.MM);
        double currentY = pos.getY(DistanceUnit.MM);
        
        double[] Forward = getForwardStrafePower(currentY, targetY)
        double[] Strafe = getStrafePower(currentX, targetX)
        double[] Rotate = getRotatePower(currentHeading, targetHeading);

            
        double[] newWheelSpeeds = new double[4];
        
        newWheelSpeeds[0] = Forward[0] + Strafe[0] + Rotate[0];
        newWheelSpeeds[1] = Forward[1] - Strafe[1] - Rotate[1];
        newWheelSpeeds[2] = Forward[2] - Strafe[2] + Rotate[2];
        newWheelSpeeds[3] = Forward[3] + Strafe[3] - Rotate[3];

        frontLeft.setPower(newWheelSpeeds[0]);
        frontRight.setPower(newWheelSpeeds[1]);
        backLeft.setPower(newWheelSpeeds[2]);
        backRight.setPower(newWheelSpeeds[3]);

        if (newWheelSpeeds[0] = 0 && newWheelSpeeds[1] = 0 && newWheelSpeeds[2] = 0 && newWheelSpeeds[3] = 0) {
            return true
        } else {
            return false
        }
    }
    
    @Override
    public void init(){

        Flywheel flywheel = new Flywheel() //Class object with functions setVelocity(rpm) and setZero()
        
        odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo")

        //Odometry Computer Configuration
        odo.setOffsets( , ); //Offsets for where the opometry pods are on the robot
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWING_ARM); //Check if correct, for what hardware ur using
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD); //to tell encoder what positive direction is

        odo.resetPosAndIMU();
        Pose2D startingPosition = new Pose2D(DistanceUnit.MM, x, y, AngleUnit.RADIANS, 0)//Set the x and y values for the starting pos (depends on where you start) and heading
        odo.setPosition(startingPosition)
        
    }
    @Override
    public void loop(){
        //Testing code
        pose2D pos = odo.getPosition();
        double currentHeading = pos.getHeading(AngleUnit.DEGREES);
        double currentX = pos.getX(DistanceUnit.MM);
        double currentY = pos.getY(DistanceUnit.MM);
        

        
        if (gamepad1.dpad_left) {
            targetX -= 200;
        }
        if (gamepad1.dpad_right) {
            targetX += 200;
        }
        
        if (gamepad1.dpad_up) {
            targetY += 200;
        }
        if (gamepad1.dpad_down) {
            targetY -= 200;
        }
        
        if (gamepad1.left_bumper) {
            targetHeading -= 10;
        }
        if (gamepad1.right_bumper) {
            targetHeading += 10;
        }


    
        
        
        telemetry.addData("Delta X:", deltaX);
        telemetry.addData("Delta Y:", deltaY);
        telemetry.addData("Delta Heading:", deltaHeading);

        if (gamepad1.a) {
            telemetry.addData("Target Position Set");
            double targetX = currentX + deltaX;
            double targetY = currentY+deltaY;
            double targetHeading = currentHeading + deltaHeading;
            
            while (!(moveRobot(targetX, targetY, targetHeading))) {
                telemetry.addLine("Moving to Position");
                if (gamepad1.y) {
                    break
                }
            }
        }

        if (gamepad1.x) {
            telemetry.addData("Target Position Reset");
            deltaX = 0;
            deltaY = 0;
            deltaHeading = 0;
        }
        
    }
