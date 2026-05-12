package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.teamcode.mechanisms.AprilTagWebcam;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

@TeleOp(name = "TrackingTest")
public class TrackingTest extends OpMode {

    private AprilTagWebcam aprilTagWebcam = new AprilTagWebcam();
    private TurretTrack turretTrack = new TurretTrack();

    private double[] step = {0, 0.1, 0.01, 0.001, 0.0001, 0.0001};
    private int stepIndex = 0;

    private Gamepad currentGamepad1 = new Gamepad();
    private Gamepad previousGamepad1 = new Gamepad();

    @Override
    public void init() {
        aprilTagWebcam.init(hardwareMap, telemetry);
        turretTrack.init(hardwareMap);

        telemetry.addLine("Initiallised");
    }

    public void start(){
        turretTrack.resetTimer();
    }

    @Override
    public void loop(){
        aprilTagWebcam.update();
        AprilTagDetection id20 = aprilTagWebcam.getTagBySpecificId(20);

        turretTrack.update(id20);

        if (id20 != null){
            telemetry.addData("cur ID", id20.id);
            telemetry.addData("Bearing", id20.ftcPose.bearing);
        }
        else{
            telemetry.addLine("No tag detected!");
        }

        // Store previous state and get current state
        previousGamepad1.copy(currentGamepad1);
        currentGamepad1.copy(gamepad1);

        // Step Index Toggle (B)
        if (currentGamepad1.b && !previousGamepad1.b) {
            stepIndex = (stepIndex + 1) % step.length;
        }

        if (currentGamepad1.dpad_left && !previousGamepad1.dpad_left) turretTrack.setKD(turretTrack.getKD() - step[stepIndex]);
        if (currentGamepad1.dpad_right && !previousGamepad1.dpad_right) turretTrack.setKD(turretTrack.getKD() + step[stepIndex]);

        // Adjust P (Dpad Up/Down)
        if (currentGamepad1.dpad_up && !previousGamepad1.dpad_up) turretTrack.setKP(turretTrack.getKP() + step[stepIndex]);
        if (currentGamepad1.dpad_down && !previousGamepad1.dpad_down) turretTrack.setKP(turretTrack.getKP() - step[stepIndex]);

        telemetry.addData("P (Up/Down)", "%.5f", turretTrack.getKP());
        telemetry.addData("d (Left/Right)", "%.5f", turretTrack.getKD());
        telemetry.addData("Step (B to cycle)", "%.5f", step[stepIndex]);
        telemetry.update();

    }
}
