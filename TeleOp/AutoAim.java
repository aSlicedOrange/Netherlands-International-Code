package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
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

    private static final int TARGET_ID = 20;

    private double flywheelTarget = 2000; // ticks/sec (tune)
    private double kP = 0.02;

    private static final int TURRET_MAX_RoTATION = 1000; // ticks (tune)
    private static final int TURRET_MIN_ROTATION = -1000; // ticks (tune)

    private static final double TURRET_TOLERANCE = 1.5;

    private static final double TICKS_PER_REV = 537.6;
    private static final double GEAR_RATIO = 1.0; //Adjust

    private double ticksPerDegree = (TICKS_PER_REV * GEAR_RATIO) / 360.0;

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
        flyL = hardwareMap.get(DcMotorEx.class, "flyL");
        flyR = hardwareMap.get(DcMotorEx.class, "flyR");
        backPlate = hardwareMap.get(Servo.class, "backPlate");

        flyR.setDirection(DcMotorEx.Direction.REVERSE);
        turret.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        turret.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        camera.init(hardwareMap, telemetry, "Webcam 1");

        waitForStart();

        while (opModeIsActive()) {

            camera.update();

            AprilTagDetection tag = camera.getTagBySpecificId(TARGET_ID);

            if(tag == null ){
                telemetry.addLine("Tag not found");
                telemetry.update();
                continue;
            }

            double range = tag.ftcPose.range;
            double bearing = tag.ftcPose.bearing;

            telemetry.addData("Range", range);
            telemetry.addData("Bearing", bearing);

            boolean turretReady = updateTurret(bearing);
            boolean flyReady = handleFlywheel();
            updateBackplate(range);

            if (turretReady && flyReady) {
                state = State.READY;
            } else {
                state = State.SPINNING_UP;
            }

            if (state == State.READY && gamepad1.a) {
                shoot();
            }

            telemetry.addData("State", state);
            telemetry.addData("Bearing", bearing);
            telemetry.addData("Range", range);
            telemetry.addData("Fly vel L", flyL.getVelocity());
            telemetry.update();
        }

        stopAll();
        camera.stop();
    }


    private boolean updateTurret(double bearing) {

        if (Math.abs(bearing) < TURRET_TOLERANCE) {
            turret.setPower(0);
            return true;
        }

        double power = bearing * kP;

        power = Math.max(-0.6, Math.min(0.6, power));

        int current = turret.getCurrentPosition();
        int target = current + (int)(bearing * ticksPerDegree);

        turret.setTargetPosition(Math.max(TURRET_MIN_ROTATION, Math.min(TURRET_MAX_RoTATION, target)));
        turret.setPower(Math.abs(power));

        return false;
    }

    private void updateBackplate(double range) {

        if (range < 30) {
            backPlate.setPosition(0.25);
        } else if (range < 55) {
            backPlate.setPosition(0.5);
        } else {
            backPlate.setPosition(0.75);
        }
    }

    private boolean handleFlywheel() {

        flyL.setVelocity(flywheelTarget);
        flyR.setVelocity(flywheelTarget);

        return Math.abs(flyL.getVelocity() - flywheelTarget) < 80 &&
               Math.abs(flyR.getVelocity() - flywheelTarget) < 80;
    }

    private void shoot() {

        // replace with shooter code

        telemetry.addLine("SHOOT!");
    }

    private void stopAll() {
        turret.setPower(0);
        flyL.setVelocity(0);
        flyR.setVelocity(0);
    }
}
