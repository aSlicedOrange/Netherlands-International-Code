package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Gamepad;

@TeleOp(name = "PIDF Tuner")
public class PIDF_Tuner extends OpMode {

    private DcMotorEx accelMotor1;
    private DcMotorEx accelMotor2;
    private DcMotor intake;
    private double TargetVel = 0, fastVel = 1600, slowVel = 1200;
    private double p = 50, f = 16;
    private double[] step = {10, 1, 0.1, 0.01, 0.001, 0.0001};
    private int stepIndex = 0;

    // Objects to track button state for "WasPressed" logic
    private Gamepad currentGamepad1 = new Gamepad();
    private Gamepad previousGamepad1 = new Gamepad();

    @Override
    public void init() {
        accelMotor1 = hardwareMap.get(DcMotorEx.class, "flywheelMotorRight");
        accelMotor2 = hardwareMap.get(DcMotorEx.class, "flywheelMotorLeft");
        
        accelMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        accelMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        
        accelMotor1.setDirection(DcMotorSimple.Direction.REVERSE);
        
        intake = hardwareMap.get(DcMotor.class, "intakeMotor");
        
        // Initial coefficients
        updatePIDF();
    }

    @Override
    public void loop() {
        // Store previous state and get current state
        previousGamepad1.copy(currentGamepad1);
        currentGamepad1.copy(gamepad1);
        
        intake.setPower((gamepad1.left_trigger > 0.1) ? -gamepad1.left_trigger : 0);

        // Velocity Toggle (Y)
        if (currentGamepad1.y && !previousGamepad1.y) {
            TargetVel = (TargetVel == fastVel) ? slowVel : fastVel;
        }

        // Step Index Toggle (B)
        if (currentGamepad1.b && !previousGamepad1.b) {
            stepIndex = (stepIndex + 1) % step.length;
        }

        // Adjust F (Dpad Left/Right)
        if (currentGamepad1.dpad_left && !previousGamepad1.dpad_left) f -= step[stepIndex];
        if (currentGamepad1.dpad_right && !previousGamepad1.dpad_right) f += step[stepIndex];

        // Adjust P (Dpad Up/Down)
        if (currentGamepad1.dpad_up && !previousGamepad1.dpad_up) p += step[stepIndex];
        if (currentGamepad1.dpad_down && !previousGamepad1.dpad_down) p -= step[stepIndex];

        // Apply tuning changes
        updatePIDF();
        accelMotor1.setVelocity(TargetVel);

        // Telemetry
        double curVel = accelMotor1.getVelocity();
        telemetry.addData("Target Velocity", TargetVel);
        telemetry.addData("Current Velocity", "%.2f", curVel);
        telemetry.addData("Error", "%.2f", TargetVel - curVel);
        telemetry.addLine("---------------------------");
        telemetry.addData("P (Up/Down)", "%.4f", p);
        telemetry.addData("F (Left/Right)", "%.4f", f);
        telemetry.addData("Step (B to cycle)", "%.4f", step[stepIndex]);
        telemetry.update();
    }

    private void updatePIDF() {
        // Correct method to set coefficients for RUN_USING_ENCODER
        accelMotor1.setVelocityPIDFCoefficients(p, 0, 0, f);
        accelMotor1.setVelocityPIDFCoefficients(p, 0, 0, f);
    }
}
