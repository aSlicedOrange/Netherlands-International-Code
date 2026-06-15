package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.mechanisms.AprilTagWebcam;

@TeleOp(name = "manual Telop")
public class manualTelop extends LinearOpMode {

    private DcMotorEx turret;
    private DcMotorEx flyL, flyR;

    private Servo backPlate;
    private Servo BlockerServo;
    private DcMotor intake;

    private DcMotor leftFrontMotor;
    private DcMotor rightFrontMotor;
    private DcMotor leftBackMotor;
    private DcMotor rightBackMotor;

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
        intake = hardwareMap.get(DcMotor.class, "intake");
        BlockerServo = hardwareMap.get(Servo.class, "BlockerServo");

        flyR.setDirection(DcMotorEx.Direction.REVERSE);
        intake.setDirection(DcMotorSimple.Direction.REVERSE);
        turret.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        turret.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        leftFrontMotor = hardwareMap.get(DcMotor.class, "lf");
        leftBackMotor = hardwareMap.get(DcMotor.class, "lb");
        rightFrontMotor = hardwareMap.get(DcMotor.class, "rf");
        rightBackMotor = hardwareMap.get(DcMotor.class, "rb");

        camera.init(hardwareMap, telemetry, "Webcam 1");

        waitForStart();

        while (opModeIsActive()) {

            //------------Vision------------//

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

            //------------Turret & Flywheel------------//

            boolean flyReady = false;

            if (gamepad1.right_bumper) {

                flyReady = activeFlywheel();
                state = State.SPINNING_UP;
                blockerServo.setPosition(0.0); //CHECK THIS VALUE

            } else {

                idleFlywheel();
                intake.setPower(0.5);
                blockerServo.setPosition(0.5);//CHECK THIS VALUE
                state = State.IDLE;

            }

            boolean turretReady = updateTurret(bearing);
            updateBackplate(range);

            //------------Shooting------------//

            if (turretReady && flyReady) {
                state = State.READY;
                intake.setPower(1);
            }

            //------------Drive------------//

            double x, y, z;

            x = gamepad1.left_stick_x;
            y = -gamepad1.left_stick_y;
            z = gamepad1.right_stick_x;

            double rfp = y + x + z;
            double lfp = y - x - z;
            double rbp = y - x + z;
            double lbp = y + x - z;

            rightFrontMotor.setPower(rfp);
            leftFrontMotor.setPower(lfp);
            rightBackMotor.setPower(rbp);
            leftBackMotor.setPower(lbp);

            //------------Telemetry------------//

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

        flyL.setVelocity(flywheelActiveTarget);
        flyR.setVelocity(flywheelActiveTarget);

        return Math.abs(flyL.getVelocity() - flywheelActiveTarget) < 80 &&
               Math.abs(flyR.getVelocity() - flywheelActiveTarget) < 80;

        //returning readyness
    }

    private void idleFlywheel() {

        flyL.setVelocity(flywheelIdleTarget);
        flyR.setVelocity(flywheelIdleTarget);
    }

    private void stopAll() {
        intake.setPower(0);
        turret.setPower(0);
        flyL.setVelocity(0);
        flyR.setVelocity(0);
    }
}
