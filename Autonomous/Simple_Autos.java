package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import java.util.ArrayList;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;


@Autonomous(name = "Simple Auto Files")
public class Simple_Autos extends LinearOpMode{
  
    private DcMotorEx frontLeft;
    private DcMotorEx frontRight;
    private DcMotorEx backLeft;
    private DcMotorEx backRight;

    private DcMotorEx intakeMotor;

    Flywheel flywheel = new Flywheel(hardwareMap);

  
    private double driveSpeed = 0.6; 
    private double slowSpeed = 0.3;

    
    static final double TICKS_PER_REV = 28;
    static final double WHEEL_DIAMETER = 104; //Millimeters
    static final double GEAR_RATIO = 20;

    public void Simple_Auto(String colour) {
      int colourM;
      if (colour.equals("Blue")) {
          colourM = 1;
      } else {
          colourM = -1;
      }
      //Simple_Auto Routine
      encoderDrive(driveSpeed, 1200);
            
      flywheel.setRPM(2000); 

      sleep(3500);
      intakeMotor.setPower(-1);
      sleep(200);
      intakeMotor.setPower(1);
            
      sleep(2500);
          
      flywheel.setZero();
      intakeMotor.setPower(0);
            
      sleep(500);
            
      strafe(driveSpeed, 500*colourM);
          
      frontLeft.setPower(0);
      frontRight.setPower(0);
      backLeft.setPower(0);
      backRight.setPower(0);
    }
    public void Strafe_Auto(String colour) {
      int colourM;
      if (colour == "Blue") {
          colourM = 1;
      } else {
          colourM = -1;
      }
      //Strafe_Auto Routine
      strafe(driveSpeed, -609.5*colourM);
            
        
      frontLeft.setPower(0);
      frontRight.setPower(0);
      backLeft.setPower(0);
      backRight.setPower(0);
    }
    public void Forward_Auto() {

      //Forward_Auto Routine
      encoderDrive(driveSpeed, 609.5);
            
        
      frontLeft.setPower(0);
      frontRight.setPower(0);
      backLeft.setPower(0);
      backRight.setPower(0);
    }

    @Override
    public void runOpMode() throws InterruptedException {

        hardwareInit(hardwareMap);

        String selectedTeam = "";
        String selectedFile = "";

        com.qualcomm.robotcore.hardware.Gamepad currentGamepad1 = new com.qualcomm.robotcore.hardware.Gamepad();
        com.qualcomm.robotcore.hardware.Gamepad previousGamepad1 = new com.qualcomm.robotcore.hardware.Gamepad();

        boolean selected = false;
        while (!(gamepad1.a) && !(selected)) {
            previousGamepad1.copy(currentGamepad1);
            currentGamepad1.copy(gamepad1);
            if (currentGamepad1.b && !previousGamepad1.b) {
                if (!(selectedFile.isEmpty())) {
                  selectedFile = "";
                } else {
                  selectedTeam = "";
                }
            }

            boolean selected = false;
          
            telemetry.addData("Selected Team: ", selectedTeam);
            telemetry.addData("Selected File: ", selectedFile);
          
            if (selectedTeam.isEmpty()) {
                telemetry.addLine("DPAD UP FOR RED");
                telemetry.addLine("DPAD DOWN FOR BLUE");
                if (currentGamepad1.dpad_up && !previousGamepad1.dpad_up) {
                    selectedTeam = "Red";
                } else if (currentGamepad1.dpad_down && !previousGamepad1.dpad_down) {
                    selectedTeam = "Blue";
                }
            } else if (selectedFile.isEmpty()) {
                telemetry.addLine("DPAD UP FOR FORWARD_AUTO");
                telemetry.addLine("DPAD RIGHT FOR SIMPLE_AUTO");
                telemetry.addLine("DPAD DOWN FOR STRAFE_AUTO");
                if (currentGamepad1.dpad_up && !previousGamepad1.dpad_up) {
                    selectedFile = "Forward_Auto";
                } else if (currentGamepad1.dpad_right && !previousGamepad1.dpad_right) {
                    selectedFile = "Simple_Auto";
                } else if (currentGamepad1.dpad_down && !previousGamepad1.dpad_down) {
                    selectedFile = "Strafe_Auto";
                }
                
            } else {
                telemetry.addLine("Are you sure with your choices?");
                telemetry.addLine("Press A to Lock In your choices.");
                telemetry.addLine("Press B to go back.");
                boolean selected = true;
            }
            telemetry.update();
          
        }
      
      
        waitForStart();

        if(opModeIsActive()) {

            switch(selectedFile) {
                case "Forward_Auto":
                  Forward_Auto();
                  break;

                case "Strafe_Auto":
                  Strafe_Auto(selectedTeam);
                  break;

                case "Simple_Auto":
                  Simple_Auto(selectedTeam);
                  break;
            }
          

            
        }
    }

    public void hardwareInit(HardwareMap hdwr){
        frontLeft = hdwr.get(DcMotorEx.class, "frontLeft");
        backLeft = hdwr.get(DcMotorEx.class, "backLeft");
        frontRight = hdwr.get(DcMotorEx.class, "frontRight");
        backRight = hdwr.get(DcMotorEx.class, "backRight");

        intakeMotor = hdwr.get(DcMotorEx.class, "intakeMotor");
        
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

      public void encoderDrive(double speed, double distanceMM) {

        int ticks = (int) (
                (distanceMM / (Math.PI * WHEEL_DIAMETER))
                * TICKS_PER_REV
                * GEAR_RATIO
        );

        int frontLeftTarget = frontLeft.getCurrentPosition() + ticks;
        int frontRightTarget = frontRight.getCurrentPosition() + ticks;
        int backLeftTarget = backLeft.getCurrentPosition() + ticks;
        int backRightTarget = backRight.getCurrentPosition() + ticks;

        frontLeft.setTargetPosition(frontLeftTarget);
        frontRight.setTargetPosition(frontRightTarget);
        backLeft.setTargetPosition(backLeftTarget);
        backRight.setTargetPosition(backRightTarget);

        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        frontLeft.setPower(speed);
        frontRight.setPower(speed);
        backLeft.setPower(speed);
        backRight.setPower(speed);

        while (opModeIsActive() &&
            (frontLeft.isBusy() || frontRight.isBusy() ||
            backLeft.isBusy() || backRight.isBusy())) {

            telemetry.addData("FL", frontLeft.getCurrentPosition());
            telemetry.addData("FR", frontRight.getCurrentPosition());
            telemetry.update();
        }

        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);

        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
}

    public void strafe(double speed, double distanceMM) {

    int ticks = (int) ((distanceMM / (Math.PI * WHEEL_DIAMETER)) * TICKS_PER_REV * GEAR_RATIO);

    frontLeft.setTargetPosition(frontLeft.getCurrentPosition() + ticks);
    frontRight.setTargetPosition(frontRight.getCurrentPosition() - ticks);
    backLeft.setTargetPosition(backLeft.getCurrentPosition() - ticks);
    backRight.setTargetPosition(backRight.getCurrentPosition() + ticks);

    frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

    frontLeft.setPower(Math.abs(speed));
    frontRight.setPower(Math.abs(speed));
    backLeft.setPower(Math.abs(speed));
    backRight.setPower(Math.abs(speed));


    while (opModeIsActive() &&
        (frontLeft.isBusy() || frontRight.isBusy() ||
        backLeft.isBusy() || backRight.isBusy())) {

        telemetry.addData("FL", frontLeft.getCurrentPosition());
        telemetry.addData("FR", frontRight.getCurrentPosition());
        telemetry.update();
    }

    frontLeft.setPower(0);
    frontRight.setPower(0);
    backLeft.setPower(0);
    backRight.setPower(0);

    frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
}

    public void rotate(double speed, double angleDegrees) {

    double robotWidth = 435;
    double wheelDistance = (robotWidth * Math.PI) * (angleDegrees / 360.0);

    int ticks = (int)((wheelDistance / (Math.PI * WHEEL_DIAMETER)) * TICKS_PER_REV * GEAR_RATIO);

    frontLeft.setTargetPosition(frontLeft.getCurrentPosition() + ticks);
    frontRight.setTargetPosition(frontRight.getCurrentPosition() - ticks);
    backLeft.setTargetPosition(backLeft.getCurrentPosition() + ticks);
    backRight.setTargetPosition(backRight.getCurrentPosition() - ticks);

    frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

    frontLeft.setPower(Math.abs(speed));
    frontRight.setPower(Math.abs(speed));
    backLeft.setPower(Math.abs(speed));
    backRight.setPower(Math.abs(speed));

    while (opModeIsActive() &&
        (frontLeft.isBusy() || frontRight.isBusy() ||
        backLeft.isBusy() || backRight.isBusy())) {

        telemetry.addData("FL", frontLeft.getCurrentPosition());
        telemetry.addData("FR", frontRight.getCurrentPosition());
        telemetry.update();
    }

    frontLeft.setPower(0);
    frontRight.setPower(0);
    backLeft.setPower(0);
    backRight.setPower(0);

    frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
}

}
