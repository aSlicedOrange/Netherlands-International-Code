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

@TeleOp(name = "Assisted TeleOp")
public class assistedTeleop extends OpMode {

    private Odometry odo; 
    private boolean yPressed = true;

    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;
    
    private DcMotor intake;
    private Flywheel flywheel;
    
    private Servo backPlate;
    private Servo BlockerServo;
    private double servoPosition;

    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;

    private static final int TARGET_ID = 20;

    private double flywheelActiveTarget = 2000; // ticks/sec (tune)
    private double flywheelIdleTarget = 800; // ticks/sec (tune)
    private double kP = 0.02;

    private static final int TURRET_MAX_RoTATION = 1000; // ticks (tune)
    private static final int TURRET_MIN_ROTATION = -1000; // ticks (tune)

    private static final double CLOSE_DISTANCE = 30; // cm (tune)
    private static final double MEDIUM_DISTANCE = 55; // cm (tune)

    private static final double TURRET_TOLERANCE = 1.5;

    private static final double TICKS_PER_REV = 537.6;
    private static final double GEAR_RATIO = 1.0; //Adjust

    private double ticksPerDegree = (TICKS_PER_REV * GEAR_RATIO) / 360.0;

    AprilTagWebcam camera = new AprilTagWebcam();

    @Override
    public void init() {
        odo = new Odometry(hardwareMap);

        frontLeft  = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft   = hardwareMap.get(DcMotor.class, "backLeft");
        backRight  = hardwareMap.get(DcMotor.class, "backRight");
        
        flywheel = new Flywheel(hardwareMap);
        intake = hardwareMap.get(DcMotor.class, "intakeMotor");
        backPlate = hardwareMap.get(Servo.class, "backPlate");
        BlockerServo = hardwareMap.get(Servo.class, "BlockerServo");

        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        
        intake.setDirection(DcMotorSimple.Direction.REVERSE);
        
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        camera.init(hardwareMap, telemetry, "Webcam 1");
    }
    

    @Override
    public void loop() {

        //------------Vision------------//

        camera.update();

        AprilTagDetection tag = camera.getTagBySpecificId(TARGET_ID);

        if(tag == null ){
            telemetry.addLine("Tag not found");
            telemetry.update();
        }
        else{

            double range = tag.ftcPose.range;
            double bearing = tag.ftcPose.bearing;

            telemetry.addData("Range", range);
            telemetry.addData("Bearing", bearing);

            updateTurret(bearing);
            updateBackplate(range); //SWITCH TO ODO RANGE LATER

        }

        //------------Turret & Flywheel------------//

        boolean flyReady = false;

        if (triangleCheck(odo.getPosition())) {
            
            boolean flyReady = activeFlywheel();     
        }
        else {
            idleFlywheel();
        }

        if (gamepad1.right_trigger > 0.1 && flyReady) {

            BlockerServo.setPosition(1); //CHECK THIS VALUE

            if BlockerServo.getPosition() > 0.9 {

                flyReady = activeFlywheel()
                intake.setPower(1);

            } else {
                intake.setPower(0);
            }

        } else {
            intake.setPower(0.5);
            BlockerServo.setPosition(0.5);//CHECK THIS VALUE
        }

        //------------Drive------------//

        odo.update();
        
        moveRobot();
        
        if (gamepad1.y && yPressed) {
            odo.reset();
            yPressed = false;
        } else if (!gamepad1.y) {
            yPressed = true;
        }
        
        
        double fvel = (Math.abs(flywheel.getVelocity()[0]) + Math.abs(flywheel.getVelocity()[1])) / 2;
        
        Pose2D pos = odo.getPosition();
        telemetry.addData("X (mm)", pos.getX(DistanceUnit.MM));
        telemetry.addData("Y (mm)", pos.getY(DistanceUnit.MM));
        telemetry.addData("Heading (deg)", pos.getHeading(AngleUnit.DEGREES));
        telemetry.addData("Target Vel: ", (6000*gamepad1.right_trigger)/60 * 28);
        telemetry.addData("Vel: ", fvel);
        telemetry.addData("Vel 1: ", Math.abs(flywheel.getVelocity()[0]));
        telemetry.addData("Vel 2: ", Math.abs(flywheel.getVelocity()[1]));
        telemetry.addData("Servo Position", backPlate.getPosition());
        telemetry.addData("Servo Target", backPlatePosition);
        telemetry.addData("bumper", gamepad1.left_bumper);
        telemetry.update();
    }

    private void updateTurret(double bearing) {

        if (Math.abs(bearing) < TURRET_TOLERANCE) {
            turret.setPower(0);
        }

        double power = bearing * kP;

        power = Math.max(-0.6, Math.min(0.6, power));

        int current = turret.getCurrentPosition();
        int target = current + (int)(bearing * ticksPerDegree);

        turret.setTargetPosition(Math.max(TURRET_MIN_ROTATION, Math.min(TURRET_MAX_RoTATION, target)));
        turret.setPower(Math.abs(power));

        //SWITCH TO PIDF CONTROL FOR TURRET LATER
    }

    private void updateBackplate(double range) {

        if (range < CLOSE_DISTANCE) {
            backPlate.setPosition(0.25);
        } else if (range < MEDIUM_DISTANCE) {
            backPlate.setPosition(0.5);
        } else {
            backPlate.setPosition(0.75);
        }

        //TUNE 
    }

    private boolean activeFlywheel() {

        flywheel.setRPM(flywheelActiveTarget);

        return Math.abs(flywheel.getRPM()[0] - flywheelActiveTarget) < 80 &&
               Math.abs(flywheel.getRPM()[1] - flywheelActiveTarget) < 80;

        //returning readyness
    }

    private void idleFlywheel() {

        flywheel.setRPM(flywheelIdleTarget);
    }

    private void stopAll() {
        intake.setPower(0);
        turret.setPower(0);
        flywheel.setZero();
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

    private boolean triangleCheck(Pose2D pos){

        double x = pos.getX(DistanceUnit.CM);
        double y = pos.getY(DistanceUnit.CM);

        // Define the vertices of the triangle
        double x1 = 100, y1 = 100; // Vertex A
        double x2 = 200, y2 = 100; // Vertex B
        double x3 = 150, y3 = 200; // Vertex C

        // Calculate the area of the triangle ABC
        double areaABC = Math.abs((x1*(y2-y3) + x2*(y3-y1) + x3*(y1-y2)) / 2.0);

        // Calculate the area of the triangle PAB
        double areaPAB = Math.abs((x*(y1-y2) + x1*(y2-y) + x2*(y-y1)) / 2.0);

        // Calculate the area of the triangle PAC
        double areaPAC = Math.abs((x*(y1-y3) + x1*(y3-y) + x3*(y-y1)) / 2.0);

        // Calculate the area of the triangle PBC
        double areaPBC = Math.abs((x*(y2-y3) + x2*(y3-y) + x3*(y-y2)) / 2.0);

        // Check if the sum of areas PAB, PAC and PBC is equal to area ABC
        return (areaABC == areaPAB + areaPAC + areaPBC);
    }
}