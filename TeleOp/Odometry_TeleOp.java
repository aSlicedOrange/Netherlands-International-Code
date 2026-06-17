package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import org.firstinspires.ftc.teamcode.Classes.Flywheel;
import org.firstinspires.ftc.teamcode.Classes.Odometry;

@TeleOp(name = "Odometry Opmode File")
public class Odometry_TeleOp extends OpMode {

    private Odometry odo; 
    private boolean yPressed = true;

    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;
    
    private DcMotor intake;
    private Flywheel flywheel;
    
    private Servo backplate;
    private Servo stopper;
    private double backplatePosition;
    private double stopperPosition;

    private double[] blueBoxPos = {2438.4, 609.6, 0};
    private double[] redBoxPos = {762, 609.6, 0};

    @Override
    public void init() {
        odo = new Odometry(hardwareMap);

        frontLeft  = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft   = hardwareMap.get(DcMotor.class, "backLeft");
        backRight  = hardwareMap.get(DcMotor.class, "backRight");
        
        flywheel = new Flywheel(hardwareMap);
        intake = hardwareMap.get(DcMotor.class, "intakeMotor");
        backplate = hardwareMap.get(Servo.class, "backplate");
        stopper = hardwareMap.get(Servo.class, "stopper");

        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        
        intake.setDirection(DcMotorSimple.Direction.REVERSE);
        
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    @Override
    public void loop() {
        odo.update();
        

        moveRobot();
        
        if (gamepad1.y && yPressed) {
            odo.reset();
            yPressed = false;
        } else if (!gamepad1.y) {
            yPressed = true;
        }
        
        if (gamepad1.right_trigger > 0.1) {
            flywheel.setRPM(6000*gamepad1.right_trigger);
        } else {
            flywheel.setZero();
        }
        
        if (gamepad1.left_trigger > 0.1 && gamepad1.a) {
            intake.setPower(-gamepad1.left_trigger);
        } else if (gamepad1.left_trigger > 0.1){
            intake.setPower(gamepad1.left_trigger);
        } else {
            intake.setPower(0);
        }

        backplatePosition = backplate.getPosition();
        stopperPosition = stopper.getPosition();

        if (gamepad1.right_bumper) { 
            backplatePosition += 0.02;
        }
        if (gamepad1.left_bumper) {
            backplatePosition -= 0.02;
        }
        
        if (gamepad1.dpad_right) { 
            stopperPosition += 0.25;
        }
        if (gamepad1.dpad_left) {
            stopperPosition -= 0.25;
        }
        backplate.setPosition(backplatePosition);
        stopper.setPosition(stopperPosition);

        
        double vel = (Math.abs(flywheel.getVelocity()[0]) + Math.abs(flywheel.getVelocity()[1])) / 2;
        
        Pose2D pos = odo.getPosition();
        telemetry.addData("X (mm)", pos.getX(DistanceUnit.MM));
        telemetry.addData("Y (mm)", pos.getY(DistanceUnit.MM));
        telemetry.addData("Heading (deg)", pos.getHeading(AngleUnit.DEGREES));
        telemetry.addData("Target Vel: ", (6000*gamepad1.right_trigger)/60 * 28);
        telemetry.addData("Vel: ", vel);
        telemetry.addData("Vel 1: ", Math.abs(flywheel.getVelocity()[0]));
        telemetry.addData("Vel 2: ", Math.abs(flywheel.getVelocity()[1]));
        telemetry.addData("Servo Position", backplate.getPosition());
        telemetry.addData("bumper", gamepad1.left_bumper);
        telemetry.update();
    }

    public void moveRobot() {
        double forward = gamepad1.left_stick_y;
        double strafe = -gamepad1.left_stick_x;
        double rotate = -gamepad1.right_stick_x;

        Pose2D pos = odo.getPosition();
        double heading = pos.getHeading(AngleUnit.RADIANS);

        double globalForward = forward * Math.cos(heading) - strafe * Math.sin(heading);
        double globalStrafe = forward * Math.sin(heading) + strafe * Math.cos(heading);

        double flPower = globalForward + globalStrafe + rotate;
        double frPower = globalForward - globalStrafe - rotate;
        double blPower = globalForward - globalStrafe + rotate;
        double brPower = globalForward + globalStrafe - rotate;

        double maxPower = Math.max(Math.abs(flPower), Math.max(Math.abs(frPower),
                          Math.max(Math.abs(blPower), Math.abs(brPower))));
        
        if (maxPower > 1.0) {
            flPower /= maxPower;
            frPower /= maxPower;
            blPower /= maxPower;
            brPower /= maxPower;
        }

        // Send power to the motors
        frontLeft.setPower(flPower);
        frontRight.setPower(frPower);
        backLeft.setPower(blPower);
        backRight.setPower(brPower);
    }
}
