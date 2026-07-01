package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import org.firstinspires.ftc.teamcode.Classes.Flywheel;
import org.firstinspires.ftc.teamcode.Classes.Odometry;

@Autonomous(name = "Odometry Test")
public class Odometry_test extends OpMode{
    
    Odometry odo;


    @Override
    public void init(){

        odo = new Odometry(hardwareMap);
        odo.setStartingPosition(0, 0, 0);
        
    }
    @Override
    public void loop(){
        //Testing code
        odo.update();
        double currentHeading = -odo.getHeading(AngleUnit.DEGREES);
        double currentX = odo.getY();
        double currentY = -odo.getX();
        
        telemetry.addData("X:", currentX);
        telemetry.addData("Y:", currentY);
        telemetry.addData("Heading:", currentHeading);
        
        telemetry.update();




        
    }
}