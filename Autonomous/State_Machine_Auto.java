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

    @Override
    public void init(){
        odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo")

        //Odometry Computer Configuration
        odo.setOffsets( , ); //Offsets for where the opometry pods are on the robot
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWING_ARM); //Check if correct, for what hardware ur using
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD); //to tell encoder what positive direction is

        odo.resetPosAndIMU();
        Pose2D startingPosition = new Pose2D(DistanceUnit.MM, x, y, AngleUnit.RADIANS, 0)//Set the x and y values for the starting pos (depends on where you start) and heading
        odo.setPosition(startingPosition)

    }
    
    public double getRotatePower(double heading) {
        
        
    }
    
    public void moveRobot(double forward, double strafe, double heading) {
        
        double rotate = ;

        
        pose2D pos = odo.getPosition();
        double heading = pos.getHeading(AngleUnit.RADIANS);

        double cosAngle = Math.cos((Math.PI / 2) - heading);
        double sinAngle = Math.sin((Math.PI / 2) - heading);

        double globalStrafe = -forward * sinAngle + strafe * cosAngle;
        double globalForward = forward * sinAngle + strafe * cosAngle;

        //Array for drive motors
        double[] newWheelSpeeds = new double[4];

        newWheelSpeeds[0] = globalForward + globalStrafe + rotate;
        newWheelSpeeds[1] = globalForward - globalStrafe - rotate;
        newWheelSpeeds[2] = globalForward - globalStrafe + rotate;
        newWheelSpeeds[3] = globalForward + globalStrafe - rotate;

        frontLeft.setPower(newWheelSpeeds[0]);
        frontRight.setPower(newWheelSpeeds[1]);
        backLeft.setPower(newWheelSpeeds[2]);
        backRight.setPower(newWheelSpeeds[3]);
    }

    @Override
    public void loop(){

    }
