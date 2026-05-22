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

@TeleOp(name = "Odometry Opmode File")
public class Odometry_TeleOp extends OpMode {

    private GoBildaPinpointDriver odo; 

    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;
    
    private DcMotor intake;
    private Flywheel flywheel;
    
    private Servo servo;
    private double servoPosition;

    @Override
    public void init() {
        odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo");

        frontLeft  = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft   = hardwareMap.get(DcMotor.class, "backLeft");
        backRight  = hardwareMap.get(DcMotor.class, "backRight");
        
        flywheel = new Flywheel(hardwareMap);
        intake = hardwareMap.get(DcMotor.class, "intakeMotor");
        servo = hardwareMap.get(Servo.class, "servo");


        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        
        intake.setDirection(DcMotorSimple.Direction.REVERSE);
        
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        odo.setOffsets(-7.7008210429995145, 4.118370236374258, DistanceUnit.INCH); 
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD); 
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD); 

        odo.resetPosAndIMU();
        Pose2D startingPosition = new Pose2D(DistanceUnit.MM, 0, 0, AngleUnit.RADIANS, 0);
        odo.setPosition(startingPosition);
    }

    @Override
    public void loop() {
        odo.update();
        

        moveRobot();
        
        if (gamepad1.right_trigger > 0.1) {
            flywheel.setPower(gamepad1.right_trigger);
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

        servoPosition = servo.getPosition();
        if (gamepad1.right_bumper) { 
            servoPosition += 0.02;
        }
        if (gamepad1.left_bumper) {
            servoPosition -= 0.02;
        }
        servo.setPosition(servoPosition);
        
        Pose2D pos = odo.getPosition();
        telemetry.addData("X (mm)", pos.getX(DistanceUnit.MM));
        telemetry.addData("Y (mm)", pos.getY(DistanceUnit.MM));
        telemetry.addData("Heading (deg)", pos.getHeading(AngleUnit.DEGREES));
        telemetry.update();
    }

    public void moveRobot() {
        double forward = gamepad1.left_stick_y;
        double strafe = -gamepad1.left_stick_x;
        double rotate = gamepad1.right_stick_x;

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
