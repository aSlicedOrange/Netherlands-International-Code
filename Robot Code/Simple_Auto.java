package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import java.util.ArrayList;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.teamcode.mechanisms.AprilTagWebcam;

@Autonomous(name = "Simple Auto Blue")
public class Simple_Auto extends LinearOpMode{

    AprilTagWebcam turretCam, sideCam;

    private Blinker control_Hub;

    private DcMotorEx frontLeft;
    private DcMotorEx frontRight;
    private DcMotorEx backLeft;
    private DcMotorEx backRight;

    private DcMotorEx intakeMotor;
    private DcMotorEx flywheelMotorRight;
    private DcMotorEx flywheelMotorLeft;

    private DcMotorEx accelMotor;
    private DcMotorEx turret;//chage to DCMotorEx

    private double BackPlateAngle;//ADD

    //private IMU imu;
    //private AngularVelocity myRobotAngularVelocity;


    //private double rotatedFromIdentity;

    private int goalTagID = 0;
    private int BLueTag = 20;
    private int OppTagID = 24;

    private double driveSpeed = 2;

    int MIN_POS = -200;
    int MAX_POS = 20;
    
    private double TargetVel = 2800;
    
    double lastPower = 0;   // make this a class variable
    double smoothing = 0.7; // 0 = instant, 1 = very smooth
    
    
    private final double flywheelTargetRPM = 6000;
    private final double flywheelP = 0.1;
    private final double flywheelF = 12.0 / flywheelTargetRPM;
    
    static final double TICKS_PER_REV = 28;
    static final double WHEEL_DIAMETER = 104; //Millimeters
    static final double GEAR_RATIO = 20;
    
    static final double TICKS_PER_REV_FLYWHEEL = 28;
    static final double GEAR_RATIO_FLYWHEEL = 0.926;

    public void setFlywheelRPM(double rpm) {
    flywheelMotorLeft.setVelocity(rpm);
    flywheelMotorRight.setVelocity(rpm);
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

    public void strafe(double speed, double distanceMM) {

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

        turretCam = new AprilTagWebcam();
        //sideCam = new AprilTagWebcam();

        hardwareInit(hardwareMap);
        telemetry.addLine("Hardware Initialised");
        turretCam.init(hardwareMap, telemetry,"Webcam 1");
        //sideCam.init(hardwareMap, telemetry, "Webcam 2");
        telemetry.addLine("Cameras Initialised");

        double driveSpeed = 0.6; 
        double slowSpeed = 0.3;

        waitForStart();

            turretCam.update();
            //sideCam.update();
            turretUpdate();
            //rotatedFromIdentity = getRotation();
            //telemetry.addData("Rotation", rotatedFromIdentity);
        if(opModeIsActive()) {
            encoderDrive(driveSpeed, 1200);
            
            setFlywheelRPM(2000); 

            sleep(3500);
            intakeMotor.setPower(-1);
            sleep(200);
            intakeMotor.setPower(1);
            
            sleep(1500);
            
            intakeMotor.setPower(-1);
            sleep(400);
            intakeMotor.setPower(1);
            sleep(3000);
          
            setFlywheelRPM(0);
            intakeMotor.setPower(0);
            
            sleep(3000);
            
            strafe(driveSpeed, 500);
          
            frontLeft.setPower(0);
            frontRight.setPower(0);
            backLeft.setPower(0);
            backRight.setPower(0);
            
        }
    }
    
    public void telemetryUpdate(){
        
        double curVel = flywheelMotorLeft.getVelocity();
        telemetry.addData("Target Velocity", TargetVel);
        telemetry.addData("Current Velocity", "%.2f", curVel);
        telemetry.addData("Error", "%.2f", TargetVel - curVel);
        telemetry.addLine("---------------------------");
        
        telemetry.update();
    }

    public void hardwareInit(HardwareMap hdwr){
        turret = hdwr.get(DcMotorEx.class, "turret");//CHANGE TO dcMotor

        frontLeft = hdwr.get(DcMotorEx.class, "frontLeft");
        backLeft = hdwr.get(DcMotorEx.class, "backLeft");
        frontRight = hdwr.get(DcMotorEx.class, "frontRight");
        backRight = hdwr.get(DcMotorEx.class, "backRight");

        intakeMotor   = hdwr.get(DcMotorEx.class, "intakeMotor");

        flywheelMotorLeft = hdwr.get(DcMotorEx.class, "flywheelMotorLeft");
        flywheelMotorRight = hdwr.get(DcMotorEx.class, "flywheelMotorRight");
        
        
        flywheelMotorLeft.setVelocityPIDFCoefficients(100, 0, 0, 16);
        flywheelMotorRight.setVelocityPIDFCoefficients(100, 0, 0, 16);

        flywheelMotorLeft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        flywheelMotorRight.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        flywheelMotorRight.setDirection(DcMotorSimple.Direction.REVERSE);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);//ADD
        flywheelMotorRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        flywheelMotorLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

    }

    public void turretUpdate() {

        AprilTagDetection goalTag = turretCam.getTagBySpecificId(goalTagID);
        int currentPos = turret.getCurrentPosition();

        double power = 0;

        if (goalTag != null) {

            double degree = goalTag.ftcPose.bearing;

            if (Math.abs(degree) > 2) {

                double kP = 0.02;
                power = degree * kP;

                double minPower = 0.05;
                if (Math.abs(power) < minPower) {
                    power = Math.signum(power) * minPower;
                }

                if ((currentPos <= MIN_POS && power < 0) ||
                    (currentPos >= MAX_POS && power > 0)) {

                    power = 0;
                
            }

        } else {
            power = 0;
        }
        power = (smoothing * lastPower) + ((1 - smoothing) * power);

        turret.setPower(power);
        lastPower = power;

        telemetry.addData("Power", power);
        telemetry.addData("Pos", currentPos);
    }
    }
}
