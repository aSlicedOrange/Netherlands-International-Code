/*
This code is basic and for having options and atleast something for auto (90% will work)

This code is for just stepping back, shooting, and strafe off the line 
This is for next to the goal start

All the functions and stuff before [while opmodeactive()] is my old code so it worked before.
*/

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;


import com.qualcomm.robotcore.util.ElapsedTime;


@Autonomous(name="Forward Auto")
public class Forward_Auto extends LinearOpMode {
    private DcMotorEx frontLeft, frontRight, backLeft, backRight;
    private DcMotor intakeMotor;
    private DcMotorEx flywheelMotorL;
    private DcMotorEx flywheelMotorR;


    // Flywheel PIDF for velocity control
    private final double flywheelTargetRPM = 6000;
    private final double flywheelP = 0.1;
    private final double flywheelF = 12.0 / flywheelTargetRPM;
    
    static final double TICKS_PER_REV = 28;
    static final double WHEEL_DIAMETER = 104; //Millimeters
    static final double GEAR_RATIO = 20;
    
    static final double TICKS_PER_REV_FLYWHEEL = 28;
    static final double GEAR_RATIO_FLYWHEEL = 0.926;
    
    static final int sleepPeriod = 5000;
    static final double motorPower = 1;
    static final double RPM = 6000;

    public void shoot() {
        setFlywheelRPM(RPM);
        intakeMotor.setPower(motorPower);
            
        sleep(sleepPeriod);
          
        setFlywheelRPM(0);
        intakeMotor.setPower(0);
    }

    public void setFlywheelRPM(double rpm) {
    double ticksPerSecond = (rpm*GEAR_RATIO_FLYWHEEL / 60.0) * TICKS_PER_REV_FLYWHEEL;
    flywheelMotorL.setVelocity(ticksPerSecond);
    flywheelMotorR.setVelocity(ticksPerSecond);
}

    public void encoderDrive(double speed, double distanceMM) {

        int ticks = (int) (
                (distanceMM / (Math.PI * WHEEL_DIAMETER))
                * TICKS_PER_REV
                * GEAR_RATIO
        );

        int frontLeftTarget = frontLeft.getCurrentPosition() + ticks;
        int frontRightTarget = frontRight.getCurrentPosition() + ticks;
        int backLeftTarget = backLeft.getCurrentPosition() + ticks;
        int backRightTarget = backRight.getCurrentPosition() + ticks;

        frontLeft.setTargetPosition(frontLeftTarget);
        frontRight.setTargetPosition(frontRightTarget);
        backLeft.setTargetPosition(backLeftTarget);
        backRight.setTargetPosition(backRightTarget);

        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        frontLeft.setPower(speed);
        frontRight.setPower(speed);
        backLeft.setPower(speed);
        backRight.setPower(speed);

        while (opModeIsActive() &&
            (frontLeft.isBusy() || frontRight.isBusy() ||
            backLeft.isBusy() || backRight.isBusy())) {

            telemetry.addData("FL", frontLeft.getCurrentPosition());
            telemetry.addData("FR", frontRight.getCurrentPosition());
            telemetry.update();
        }

        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);

        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
}

    public void strafe(double speed, double distanceMM) { // positive, moves to the right, left, oppo

    int ticks = (int) ((distanceMM / (Math.PI * WHEEL_DIAMETER)) * TICKS_PER_REV * GEAR_RATIO);

    frontLeft.setTargetPosition(frontLeft.getCurrentPosition() + ticks);
    frontRight.setTargetPosition(frontRight.getCurrentPosition() - ticks);
    backLeft.setTargetPosition(backLeft.getCurrentPosition() - ticks);
    backRight.setTargetPosition(backRight.getCurrentPosition() + ticks);

    frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

    frontLeft.setPower(Math.abs(speed));
    frontRight.setPower(Math.abs(speed));
    backLeft.setPower(Math.abs(speed));
    backRight.setPower(Math.abs(speed));


    while (opModeIsActive() &&
        (frontLeft.isBusy() || frontRight.isBusy() ||
        backLeft.isBusy() || backRight.isBusy())) {

        telemetry.addData("FL", frontLeft.getCurrentPosition());
        telemetry.addData("FR", frontRight.getCurrentPosition());
        telemetry.update();
    }

    frontLeft.setPower(0);
    frontRight.setPower(0);
    backLeft.setPower(0);
    backRight.setPower(0);

    frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
}

    public void rotate(double speed, double angleDegrees) {

    double robotWidth = 435;
    double wheelDistance = (robotWidth * Math.PI) * (angleDegrees / 360.0);

    int ticks = (int)((wheelDistance / (Math.PI * WHEEL_DIAMETER)) * TICKS_PER_REV * GEAR_RATIO);

    frontLeft.setTargetPosition(frontLeft.getCurrentPosition() + ticks);
    frontRight.setTargetPosition(frontRight.getCurrentPosition() - ticks);
    backLeft.setTargetPosition(backLeft.getCurrentPosition() + ticks);
    backRight.setTargetPosition(backRight.getCurrentPosition() - ticks);

    frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

    frontLeft.setPower(Math.abs(speed));
    frontRight.setPower(Math.abs(speed));
    backLeft.setPower(Math.abs(speed));
    backRight.setPower(Math.abs(speed));

    while (opModeIsActive() &&
        (frontLeft.isBusy() || frontRight.isBusy() ||
        backLeft.isBusy() || backRight.isBusy())) {

        telemetry.addData("FL", frontLeft.getCurrentPosition());
        telemetry.addData("FR", frontRight.getCurrentPosition());
        telemetry.update();
    }

    frontLeft.setPower(0);
    frontRight.setPower(0);
    backLeft.setPower(0);
    backRight.setPower(0);

    frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
}

    @Override
    public void runOpMode() throws InterruptedException {

    
        frontLeft  = hardwareMap.get(DcMotorEx.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotorEx.class, "frontRight");
        backLeft   = hardwareMap.get(DcMotorEx.class, "backLeft");
        backRight  = hardwareMap.get(DcMotorEx.class, "backRight");

        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
        flywheelMotorL = hardwareMap.get(DcMotorEx.class, "flywheelMotorLeft");
        flywheelMotorR = hardwareMap.get(DcMotorEx.class, "flywheelMotorRight");
        
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);
        intakeMotor.setDirection(DcMotor.Direction.REVERSE);

        flywheelMotorL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheelMotorR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        
        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(flywheelP, 0, 0, flywheelF); 
        flywheelMotorL.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        flywheelMotorR.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);

        
        double driveSpeed = 0.6; 
        double slowSpeed = 0.3;

        telemetry.addLine("Initialized!");
        telemetry.update();
        waitForStart();

        if(opModeIsActive()) {
            
            encoderDrive(driveSpeed, 609.5);
            
        
            frontLeft.setPower(0);
            frontRight.setPower(0);
            backLeft.setPower(0);
            backRight.setPower(0);
            
            intakeMotor.setPower(0);
            flywheelMotorL.setPower(0);
            flywheelMotorR.setPower(0);
        }
    }
}
