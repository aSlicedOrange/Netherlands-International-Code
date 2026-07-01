package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.mechanisms.AprilTagWebcam;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.Classes.Flywheel;

/*

Methods:
public void setRPM(double rpm)
public void setPower(double power)
public double[] getVelocity()
public void setZero()
public void updatePIDF()
*/

public class Turret {
    
    private DcMotorEx turret;
    private DcMotorEx flyL, flyR;
    private Servo backPlate;
    private Telemetry telemetry;

    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;

    private static final int TARGET_ID = 20;

    private double flywheelTarget = 0; // ticks/sec (tune)
    private double kP = 0.03;

    private static final int TURRET_MAX_ROTATION = 300; // ticks (tune)
    private static final int TURRET_MIN_ROTATION = -300; // ticks (tune)
    private static final double TURRET_TOLLERANCE = 0.5;

    private static final double TICKS_PER_REV = 228;
    private static final double GEAR_RATIO = 2; //Adjust
    
    public int dist = 2;

    private double ticksPerDegree = (TICKS_PER_REV * GEAR_RATIO) / 360.0;
    
    AprilTagWebcam camera = new AprilTagWebcam();
    
    public void init(HardwareMap hardwareMap, Telemetry telemetry) {
        turret = hardwareMap.get(DcMotorEx.class, "turret");
        flyL = hardwareMap.get(DcMotorEx.class, "flywheelMotorLeft");
        flyR = hardwareMap.get(DcMotorEx.class, "flywheelMotorRight");
        backPlate = hardwareMap.get(Servo.class, "backplate");

        flyR.setDirection(DcMotorEx.Direction.REVERSE);

        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        turret.setPower(0);

        camera.init(hardwareMap, telemetry, "Webcam 1");
    }
    
    public Turret(HardwareMap hardwareMap, Telemetry telemetry) {
      init(hardwareMap, telemetry);
    }
    
    public void update(){
        
        double bearing = 0;

        camera.update();

        AprilTagDetection tag = camera.getTagBySpecificId(TARGET_ID);
        if(tag != null){

            double range = tag.ftcPose.range;
            bearing = tag.ftcPose.bearing;
            updateBackplate(range);

        }

        updateTurret(bearing);
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
    }

    private void updateBackplate(double range) {

        if (range < 0.3) {
            backPlate.setPosition(0.4);
            dist = 0;
        } else if (range < 0.6) {
            backPlate.setPosition(0.4);
            dist = 1;
        } else {
            backPlate.setPosition(0.2);
            dist = 2;
        }
    }

    public void stopAll() {
        turret.setPower(0);
        camera.stop();
    }
    
    public void shoot(Flywheel flywheel, DcMotor intake, Servo stopper, double FVel) {
        flywheel.setRPM(FVel);
            
        double vel = (Math.abs(flywheel.getVelocity()[0]) + Math.abs(flywheel.getVelocity()[1])) / 2;
            
        if (vel > 1500){
            if (stopper.getPosition() > 0.3) {
                stopper.setPosition(stopper.getPosition()-0.05);
            }
            if(stopper.getPosition() < 0.4){
                intake.setPower(1);
            }
            else{
                intake.setPower(0);
            }
        }
        else{
            stopper.setPosition(1);
        }
        
    }
}
