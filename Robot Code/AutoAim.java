package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.State;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.mechanisms.AprilTagWebcam;

@TeleOp(name = "Auto Aim Test")
public class AutoAim extends LinearOpMode {

    private DcMotorEx turret;
    private DcMotorEx flyL, flyR;
    private Servo backPlate;

    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;

    private static final int TARGET_ID = 0;

    private double flywheelTarget = 0; // ticks/sec (tune)
    private double kP = 0.03;

    private static final int TURRET_MAX_ROTATION = 300; // ticks (tune)
    private static final int TURRET_MIN_ROTATION = -300; // ticks (tune)
    private static final double TURRET_TOLLERANCE = 0.5;

    private static final double TICKS_PER_REV = 228;
    private static final double GEAR_RATIO = 2; //Adjust

    private double ticksPerDegree = (TICKS_PER_REV * GEAR_RATIO) / 360.0;

    private double prevRange = 0;


    enum State {
        IDLE,
        SPINNING_UP,
        READY
    }

    AprilTagWebcam camera = new AprilTagWebcam();

    State state = State.IDLE;

    @Override
    public void runOpMode() {

        turret = hardwareMap.get(DcMotorEx.class, "turret");
        flyL = hardwareMap.get(DcMotorEx.class, "flywheelMotorLeft");
        flyR = hardwareMap.get(DcMotorEx.class, "flywheelMotorRight");
        backPlate = hardwareMap.get(Servo.class, "backplate");

        flyR.setDirection(DcMotorEx.Direction.REVERSE);

        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        turret.setPower(0);

        camera.init(hardwareMap, telemetry, "Webcam 1");

        waitForStart();

        while (opModeIsActive()) {

            double range = prevRange;
            double bearing = 0;

            camera.update();

            AprilTagDetection tag = camera.getTagBySpecificId(TARGET_ID);

            if(tag == null ){
                telemetry.addLine("Tag not found");
                telemetry.update();

            }
            else{

                range = tag.ftcPose.range;
                bearing = tag.ftcPose.bearing;

                telemetry.addData("Range", range);
                telemetry.addData("Bearing", bearing);

            }

            updateTurret(bearing);
            updateBackplate(range);
            handleFlywheel();

            prevRange = range;

            boolean turretReady = Math.abs(bearing) < TURRET_TOLLERANCE;
            boolean flyReady = isFlywheelReady();

            if (turretReady && flyReady) {
                state = State.READY;
            } else {
                state = State.SPINNING_UP;
            }

            if (state == State.READY && gamepad1.a) {
                shoot();
            }

            telemetry.addData("State", state);
            telemetry.addData("Fly vel L", flyL.getVelocity());
            telemetry.update();
        }

        stopAll();
        camera.stop();
    }


    private void updateTurret(double bearing) {

        if (Math.abs(bearing) < TURRET_TOLLERANCE) {
            turret.setPower(0);
            return;
        }

        double power = bearing * kP;

        power = Math.max(-0.8, Math.min(0.8, power));

        double currentPos = turret.getCurrentPosition();

        if (currentPos >= TURRET_MAX_ROTATION && power > 0) {
            power = 0;
        }

        if (currentPos <= TURRET_MIN_ROTATION && power < 0) {
            power = 0;
        }
        
        turret.setPower(power);

        telemetry.addData("pwr",power);
    }

    private void updateBackplate(double range) {

        if (range < 0.2) {
            backPlate.setPosition(0.25);
        } else if (range < 0.4) {
            backPlate.setPosition(0.5);
        } else {
            backPlate.setPosition(0.75);
        }
    }

    private void handleFlywheel() {

        flyL.setVelocity(flywheelTarget);
        flyR.setVelocity(flywheelTarget);
    }

    private boolean isFlywheelReady() {
        return Math.abs(flyL.getVelocity() - flywheelTarget) < 80 &&
               Math.abs(flyR.getVelocity() - flywheelTarget) < 80;
    }

    private void shoot() {

        // replace with shooter code

        telemetry.addLine("SHOOT!");
    }

    //private AprilTagDetection getTag(int id) {

        //for (AprilTagDetection d : aprilTag.getDetections()) {
            //if (d.id == id) return d;
        //}
        //return null;
    //}

    private void stopAll() {
        turret.setPower(0);
        flyL.setVelocity(0);
        flyR.setVelocity(0);
    }
}