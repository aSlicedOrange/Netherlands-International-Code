package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import java.util.ArrayList;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.teamcode.mechanisms.AprilTagWebcam;

@TeleOp(name = "Camera Test")
public class cameraTest extends OpMode{

    AprilTagWebcam turretCam, sideCam;


    private DcMotorEx turret;


    private boolean active = true;

    private int goalTagID = 583;
    private int BLueTag = 20;
    private int OppTagID = 24;


    int MIN_POS = -200;
    int MAX_POS = 20;
    
    
    double lastPower = 0;   // make this a class variable
    double smoothing = 0.7; // 0 = instant, 1 = very smooth


    @Override
    public void init(){

        turretCam = new AprilTagWebcam();


        hardwareInit(hardwareMap);
        telemetry.addLine("Hardware Initialised");
        turretCam.init(hardwareMap, telemetry,"Webcam 1");
        telemetry.addLine("Cameras Initialised");

    }

    @Override
    public void loop(){

        turretCam.update();
        turretUpdate();


    }
    


    public void hardwareInit(HardwareMap hdwr){
        turret = hdwr.get(DcMotorEx.class, "turret");//CHANGE TO dcMotor
        
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);//ADD

    }

    public void turretUpdate() {

        AprilTagDetection goalTag = turretCam.getTagBySpecificId(goalTagID);
        
        if (goalTag != null) {
            
            int currentPos = turret.getCurrentPosition();

            double power = 0;

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