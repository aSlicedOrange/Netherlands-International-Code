package org.firstinspires.ftc.teamcode.Testing;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;


@TeleOp(name = "Flywheel PIDF Tuning")
public class Flywheel_PIDF_Tuning extends OpMode {


    private Flywheel flywheel;
    private double targetRPM = 2000;


    @Override
    public void init() {
        flywheel = new Flywheel(hardwareMap);
    }

    @Override
    public void loop() {
    
        if (gamepad1.right_trigger > 0.1) {
            flywheel.setVelocity(targetRPM);
        } else {
            flywheel.setZero();
        }
    }
    
    
    
    }
}
