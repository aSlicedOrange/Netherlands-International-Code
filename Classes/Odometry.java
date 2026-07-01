package org.firstinspires.ftc.teamcode.Classes;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/*
Utility Class for Odometry functionality

To use make an object of the class and pass in hardwareMap, then use the methods below

Example:
private Odometry odo; (Class attribute)

odo = new Odometry(hardwareMap); (Inside Init)
You may want to set the starting position in auto and leave it to be the same in teleop

Methods:
public void setStartingPosition(double X, double Y, double Heading)
public void update()
public void reset()
public Pose2D getPosition()
public double getX()
public double getHeading(AngleUnit Unit)
*/

public class Odometry {
    private GoBildaPinpointDriver odo; 
    private Pose2D pos;

    public Odometry(HardwareMap hardwareMap) {
        init(hardwareMap);
    }
    public void init(HardwareMap hardwareMap) {
        odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo");

        odo.setOffsets(-7.7008210429995145, 4.118370236374258, DistanceUnit.INCH); 
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD); 
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED, GoBildaPinpointDriver.EncoderDirection.REVERSED); 

    }
    
    public void setStartingPosition(double X, double Y, double Heading) {
        Pose2D startingPosition = new Pose2D(DistanceUnit.MM, X, Y, AngleUnit.DEGREES, Heading);
        odo.setPosition(startingPosition);
        pos = startingPosition;
    }
    
    public void update() {
        odo.update();
        pos = odo.getPosition();
    }

    public void reset() {
        odo.resetPosAndIMU();
    }
    
    public void resetFieldCentric() {
        Pose2D resetPosition = new Pose2D(DistanceUnit.MM, pos.getX(DistanceUnit.MM), pos.getY(DistanceUnit.MM), AngleUnit.DEGREES, 0);
        odo.setPosition(resetPosition);
    }
  
    public Pose2D getPosition() {
        return pos;
    }

    public double getX() {
        return pos.getX(DistanceUnit.MM);
    }

    public double getY() {
        return pos.getY(DistanceUnit.MM);
    }

    public double getHeading(AngleUnit Unit) {
        return pos.getHeading(Unit);
    }
}