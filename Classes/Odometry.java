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

odo = new Odometry(hardwareMap, InitialX, InitialY, initialHeading); (Inside Init)

Methods:
public void update()
public Pose2D getPosition()
public double getX()
public double getHeadingRadians()
public double getHeadingDegrees()
*/

public class Odometry {
    private GoBildaPinpointDriver odo; 
    private Pose2D pos;

    public Odometry(HardwareMap hardwareMap, double initialX, double initialY, double initialHeading) {
        init(hardwareMap, initialX, initialY, initialHeading);
    }
    public void init(HardwareMap hardwareMap, double initialX, double initialY, double initialHeading) {
        odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo");

        odo.setOffsets(-7.7008210429995145, 4.118370236374258, DistanceUnit.INCH); 
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD); 
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD); 

        odo.resetPosAndIMU();
        Pose2D startingPosition = new Pose2D(DistanceUnit.MM, initialX, initialY, AngleUnit.RADIANS, initialHeading);
        odo.setPosition(startingPosition);
        
        pos = startingPosition;
    }

    public void update() {
        odo.update();
        pos = odo.getPosition();
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

    public double getHeadingRadians() {
        return pos.getHeading(AngleUnit.RADIANS);
    }
    
    public double getHeadingDegrees() {
        return pos.getHeading(AngleUnit.DEGREES);
    }
}
