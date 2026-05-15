/* 
---------------------------------------
File is finished

Needs values to be put in

Needs gobilda repository
---------------------------------------
*/

/*
It needs the gobilda github repository to work
This is field centric drive teleop with the odometry pods
*/


package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;


import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.teamcode.mechanisms.AprilTagWebcam;

@TeleOp(name = "Odometry Opmode File")
public class Odometry_TeleOp extends OpMode {

    GoBildaPinpointDriver odo; //Get the gobilda repository

    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;


    @Override
    public void init() {
        odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo")

        frontLeft  = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft   = hardwareMap.get(DcMotor.class, "backLeft");
        backRight  = hardwareMap.get(DcMotor.class, "backRight");


        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        //Odometry Computer Configuration
        odo.setOffsets( , ); //Offsets for where the opometry pods are on the robot
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWING_ARM); //Check if correct, for what hardware ur using
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD); //to tell encoder what positive direction is

        odo.resetPosAndIMU();
        Pose2D startingPosition = new Pose2D(DistanceUnit.MM, x, y, AngleUnit.RADIANS, 0)//Set the x and y values for the starting pos (depends on where you start) and heading
        odo.setPosition(startingPosition)
    }

    public void moveRobot() {

        double forward = -gamepad1.left_stick_y;
        double strafe = -gamepad1.left_stick_x;
        double rotate = -gamepad1.right_stick_x;

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
    public void loop() {
        moveRobot();

        //Gets the new position from pods every loop
        Pose2D pos = odo.getPosition();
    }
