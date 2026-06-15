package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorEx;


@TeleOp(name = "curve graphing")
public class curveGraphing extends OpMode {

    private double step = 0.05;

    private Gamepad currentGamepad1 = new Gamepad();
    private Gamepad previousGamepad1 = new Gamepad();

    private Servo backPlate;

    @Override
    public void init() {

        backPlate = hardwareMap.get(Servo.class, "backPlate");

        telemetry.addLine("Initiallised");
    }

    @Override
    public void loop(){
        
        // Store previous state and get current state
        previousGamepad1.copy(currentGamepad1);
        currentGamepad1.copy(gamepad1);

        double servoPos = backPlate.getPosition();

        // Adjust P (Dpad Up/Down)
        if (currentGamepad1.dpad_up && !previousGamepad1.dpad_up) servoPos += step;
        if (currentGamepad1.dpad_down && !previousGamepad1.dpad_down) servoPos -= step;

        backPlate.setPosition(servoPos);

        telemetry.addData("ServoPos", "%.2f", backPlate.getPosition());
        telemetry.update();

    }
}
