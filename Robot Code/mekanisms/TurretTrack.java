package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.ElapsedTime;

public class TurretTrack {

    private DcMotorEx turret;
    private double KP = 0.0001;
    private double KD = 0;
    private double goalX = 0;
    private  double lastErr = 0;
    private double Tolerance = 0.2;
    private final double MAX_POWER = 0.6;
    private double power = 0;
    private ElapsedTime timer = new ElapsedTime();

    public void init(HardwareMap hwMap){
        turret = hwMap.get(DcMotorEx.class, "turret");
        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void setKP(double newKP){
        KP = newKP;
    }

    public double getKP() {
        return KP;
    }

    public void setKD(double newKD) {
        KD = newKD;
    }

    public double getKD() {
        return KD;
    }

    public void resetTimer(){
        timer.reset();
    }

    public void update(AprilTagDetection curID){
        double deltaTime = timer.seconds();
        timer.reset();

        if(curID == null){
            turret.setPower(0);
            lastErr = 0;
            return;
        }


        double err = goalX - curID.ftcPose.bearing;
        double pTerm = err * KP;

        double dTerm = 0;
        if(deltaTime > 0){
            deltaTime = Math.max(deltaTime, 0.01);
            dTerm = ((err - lastErr) / deltaTime) * KD;
        }

        if (Math.abs(err) < Tolerance){
            power = 0;
        }
        else{
            power = Range.clip(pTerm + dTerm, -MAX_POWER, MAX_POWER);
        }

        //ADD SAFETY CHECK!!

        turret.setPower(power);
        lastErr = err;
    }
}
