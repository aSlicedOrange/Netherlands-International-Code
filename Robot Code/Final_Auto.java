package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Classes.Flywheel;
import org.firstinspires.ftc.teamcode.Classes.Odometry;
import org.firstinspires.ftc.teamcode.Classes.Movement;
import org.firstinspires.ftc.teamcode.mechanisms.Turret;

@Autonomous(name = "Final Auto")
public class Final_Auto extends OpMode {

    private Odometry odo;
    private Movement movement;
    private Turret turret;
    
    private boolean shootZone = true;
    private int loopCount = 0;

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
    
    private double FVel = 6000;

    private double[] blueBoxPos = {2438.4, 609.6, 0};
    private double[] redBoxPos = {762, 609.6, 0};
    
    private ElapsedTime shootTimer;
    int state = 0;

    @Override
    public void init() {
        odo = new Odometry(hardwareMap);
        odo.setStartingPosition(0, 0, 0);
        movement = new Movement(hardwareMap, odo);
        turret = new Turret(hardwareMap, telemetry);
        shootTimer = new ElapsedTime();

        frontLeft  = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft   = hardwareMap.get(DcMotor.class, "backLeft");
        backRight  = hardwareMap.get(DcMotor.class, "backRight");
        
        flywheel = new Flywheel(hardwareMap);
        intake = hardwareMap.get(DcMotor.class, "intakeMotor");
        backplate = hardwareMap.get(Servo.class, "backplate");
        stopper = hardwareMap.get(Servo.class, "stopper");

        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);
        
        intake.setDirection(DcMotorSimple.Direction.REVERSE);
        
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    @Override
    public void start() {
        shootTimer.reset();
        odo.reset();
    }
    
    @Override
    public void loop() {
        odo.update();
        
        turret.update();
        
        double vel = (Math.abs(flywheel.getVelocity()[0]) + Math.abs(flywheel.getVelocity()[1])) / 2;
        
        if (state == 0) {
            flywheel.setRPM(6000);
            movement.moveRobot(200, 0, 0);
            if (shootTimer.milliseconds() > 1500) {
                shootTimer.reset();
                state += 1;
            }
        }
        
        if (state == 1) {
            turret.shoot(flywheel, intake, stopper, FVel);
            if (vel > 1500) {
                if (shootTimer.milliseconds() > 2500) {
                    state += 1;
                }
            } else {
                shootTimer.reset();
            }
            backLeft.setPower(0);
            backRight.setPower(0);
            frontRight.setPower(0);
            frontLeft.setPower(0);
        }
        
        if (state == 2) {
            if (movement.moveRobot(700, 150, 0)) {
                state += 1;
            }
            flywheel.setZero();
            intake.setPower(0);
        }

        if (state == 3) {
            intake.setPower(0.75);
            if (movement.moveRobot(700, 500, 0)) {
                state += 1;
            }
        }

        if (state == 4) {
            if (shootZone) {
                if (movement.moveRobot(200, 0, 0)) {
                    turret.shoot(flywheel, intake, stopper, FVel);
                    if (vel > 1500) {
                        if (shootTimer.milliseconds() > 2500) {
                            shootZone = false;
                            loopCount += 1;
                        }
                    } else {
                        shootTimer.reset();
                    }
                } else {
                    flywheel.setRPM(6000);
                    intake.setPower(0.75);
                }
            
            } else {
                flywheel.setZero();
                intake.setPower(0.75);
                if (movement.moveRobot(0, 900, 0)) {
                    shootZone = true;
                }
            }
        }

        telemetry.addData("State", state);
        telemetry.addData("loopCount", loopCount);
        telemetry.update();
    }
}
