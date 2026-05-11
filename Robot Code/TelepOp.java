package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.teamcode.mechanisms.AprilTagWebcam;

@TeleOp(name = "TeleOp 2026")
public class TelepOp extends OpMode{

    AprilTagWebcam turretCam, sideCam;

    private Blinker control_Hub;

    private DcMotor leftFrontMotor;
    private DcMotor rightFrontMotor;
    private DcMotor leftBackMotor;
    private DcMotor rightBackMotor;

    private DcMotorEx intakeMotor;
    private DcMotorEx flywheelMotorRight;
    private DcMotorEx flywheelMotorLeft;

    private DcMotorEx accelMotor;
    private DcMotorEx turret;

    private double BackPlateAngle;//ADD

    //private IMU imu;
    //private AngularVelocity myRobotAngularVelocity;

    private boolean active = true;

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


    @Override
    public void init(){

        turretCam = new AprilTagWebcam();
        //sideCam = new AprilTagWebcam();

        hardwareInit(hardwareMap);
        telemetry.addLine("Hardware Initialised");
        turretCam.init(hardwareMap, telemetry,"Webcam 1");
        //sideCam.init(hardwareMap, telemetry, "Webcam 2");
        telemetry.addLine("Cameras Initialised");


        telemetry.addLine("Press to start...");
    }

    @Override
    public void loop(){

        if (active){

            turretCam.update();
            //sideCam.update();
            turretUpdate();
            //rotatedFromIdentity = getRotation();
            //telemetry.addData("Rotation", rotatedFromIdentity);

            double x, y, z;

            x = gamepad1.left_stick_x;
            y = gamepad1.left_stick_y;
            z = gamepad1.right_stick_x;

            double rfa = y + x + z;
            double lfa = y - x - z;
            double rba = y - x + z;
            double lba = y + x - z;

            rightFrontMotor.setPower(rfa);
            leftFrontMotor.setPower(lfa);
            rightBackMotor.setPower(rba);
            leftBackMotor.setPower(lba);

            //Intake
            boolean reverseIntake = gamepad1.a;
            if (reverseIntake) {
                intakeMotor.setPower(-(gamepad1.left_trigger));
            } else {
                intakeMotor.setPower(gamepad1.left_trigger);
            }
            // Flywheel
            if (gamepad1.right_trigger > 0.1) {
                flywheelMotorRight.setVelocity(TargetVel);
                flywheelMotorLeft.setVelocity(TargetVel);
            }
            else{
                flywheelMotorRight.setVelocity(0);
                flywheelMotorLeft.setVelocity(0);
            }
            
            telemetryUpdate();

        }
        else{telemetry.addLine("Press Dpad-Down to activate");}
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

        leftFrontMotor = hdwr.get(DcMotor.class, "frontLeft");
        leftBackMotor = hdwr.get(DcMotor.class, "backLeft");
        rightFrontMotor = hdwr.get(DcMotor.class, "frontRight");
        rightBackMotor = hdwr.get(DcMotor.class, "backRight");

        intakeMotor   = hdwr.get(DcMotorEx.class, "intakeMotor");

        flywheelMotorLeft = hdwr.get(DcMotorEx.class, "flywheelMotorLeft");
        flywheelMotorRight = hdwr.get(DcMotorEx.class, "flywheelMotorRight");
        
        
        flywheelMotorLeft.setVelocityPIDFCoefficients(100, 0, 0, 16);
        flywheelMotorRight.setVelocityPIDFCoefficients(100, 0, 0, 16);

        flywheelMotorLeft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        flywheelMotorRight.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        flywheelMotorRight.setDirection(DcMotorSimple.Direction.REVERSE);

        leftFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        
        leftFrontMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBackMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);//ADD
        flywheelMotorRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        flywheelMotorLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);


        //imu = hardwareMap.get(IMU.class, "IMU");
        //IMU.Parameters  parameters;
        //parameters = new IMU.Parameters(
                //new RevHubOrientationOnRobot(
                        //RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        //RevHubOrientationOnRobot.UsbFacingDirection.FORWARD //Change for current robot
                //)
        //);
        //imu.resetYaw();

        //imu.initialize(parameters);

        //myRobotAngularVelocity = imu.getRobotAngularVelocity(AngleUnit.DEGREES);

    }

    public void turretUpdate() {

        AprilTagDetection goalTag = turretCam.getTagBySpecificId(goalTagID);
        
        if (goalTag != null) {
            
            int currentPos = turret.getCurrentPosition();

            double power = 0; //Why does it set to 0 at the start

            double degree = goalTag.ftcPose.bearing;

            if (Math.abs(degree) > 2) {

                double kP = 0.02;
                power = degree * kP;

                double minPower = 0.05;
                if (Math.abs(power) < minPower) {
                    power = Math.signum(power) * minPower; //Why does it * min power shouldent it set to 0
                }

                if ((currentPos <= MIN_POS && power < 0) ||
                    (currentPos >= MAX_POS && power > 0)) { //Why is it currentPos <= MIN_POS and then a -ve power

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

    //public double getRotation(){
        //AprilTagDetection centerTag = sideCam.getTagBySpecificId(centerTagID);
        //AprilTagDetection goalTag = sideCam.getTagBySpecificId(goalTagID);
        //AprilTagDetection OppTag = sideCam.getTagBySpecificId(OppTagID);

        //if (centerTag == null && goalTag == null && OppTag == null){
            //double zRotationRate = myRobotAngularVelocity.zRotationRate;
            //return (rotatedFromIdentity + zRotationRate);
        //}

        //ArrayList<Double> angles = new ArrayList<Double>();

        //if(centerTag != null){
            //angles.add(centerTag.ftcPose.bearing - 90);
        //}


        //if(goalTag != null){
            //angles.add(goalTag.ftcPose.bearing + 45);
        //}


        //if(OppTag != null){
            //angles.add(OppTag.ftcPose.bearing - 45);
        //}

        //double totalAngle = 0;
        //for (int i = 0; i < angles.size(); i++) {
            //totalAngle += angles.get(i);
        //}

        //return (totalAngle/angles.size());
        //}
    }
}
