package org.firstinspires.ftc.teamcode.Testing;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "Test TeleOp Fixed")
public class TestTeleop extends OpMode {

    private DcMotor lf;
    private DcMotor lb;
    private DcMotor rf;
    private DcMotor rb;
    
    private DcMotor flywheel;
    private DcMotor intake;

    @Override
    public void init() {
        // Hardware Mapping
        lf = hardwareMap.get(DcMotor.class, "lf");
        lb = hardwareMap.get(DcMotor.class, "lb");
        rf = hardwareMap.get(DcMotor.class, "rf");
        rb = hardwareMap.get(DcMotor.class, "rb");

        intake = hardwareMap.get(DcMotor.class, "intake");
        flywheel = hardwareMap.get(DcMotor.class, "flywheel");
        
        // Zero Power Behaviors
        lf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        
        flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        
        // Correct Mecanum Direction (Reverse the entire left side)
        lf.setDirection(DcMotorSimple.Direction.REVERSE);
        lb.setDirection(DcMotorSimple.Direction.REVERSE);
        rf.setDirection(DcMotorSimple.Direction.FORWARD);
        rb.setDirection(DcMotorSimple.Direction.FORWARD);
        
        // Mechanisms Direction
        flywheel.setDirection(DcMotorSimple.Direction.REVERSE); 
        intake.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    @Override
    public void loop() {
        double y = -gamepad1.left_stick_y; 
        double x = gamepad1.left_stick_x;
        double z = gamepad1.right_stick_x;

        // 2. Apply proper mathematical Deadzone using Math.abs()
        if (Math.abs(y) < 0.1) y = 0;
        if (Math.abs(x) < 0.1) x = 0;
        if (Math.abs(z) < 0.1) z = 0;
        
        // 3. Standard Mecanum Kinematics Formulas
        double lfa = y + x + z;
        double lba = y - x + z;
        double rfa = y - x - z;
        double rba = y + x - z;

        // 4. Clip/Normalize Powers to stay within the [-1.0, 1.0] safe limit
        double max = Math.max(Math.max(Math.abs(lfa), Math.abs(lba)), 
                               Math.max(Math.abs(rfa), Math.abs(rba)));
        if (max > 1.0) {
            lfa /= max;
            lba /= max;
            rfa /= max;
            rba /= max;
        }

        // Send powers to drivetrain
        lf.setPower(lfa);
        lb.setPower(lba);
        rf.setPower(rfa);
        rb.setPower(rba);
        
        // 5. Flywheel Control (Right Trigger)
        if (gamepad1.right_trigger > 0.1) {
            flywheel.setPower(gamepad1.right_trigger);
        } else {
            flywheel.setPower(0);
        }
        
        // 6. Intake Control (Left Trigger toggles Outtake with X button)
        if (gamepad1.left_trigger > 0.1) {
            if (gamepad1.x) {
                intake.setPower(-gamepad1.left_trigger); // Reverse power for outtake
            } else {
                intake.setPower(gamepad1.left_trigger);  // Normal intake
            }
        } else {
            intake.setPower(0);
        }
    }
}
