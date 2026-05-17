package org.firstinspires.ftc.teamcode.Classes;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class Flywheel {
    private GoBildaPinpointDriver odo; 
    private Pose2D pos;

  public Flywheel(HardwareMap hardwareMap) {
      init(hardwareMap);
  }
  public void init(HardwareMap hardwareMap) {
      odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo");

      odo.setOffsets(-7.7008210429995145, 4.118370236374258, DistanceUnit.INCH); 
      odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD); 
      odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD); 

      odo.resetPosAndIMU();
      Pose2D startingPosition = new Pose2D(DistanceUnit.MM, 0, 0, AngleUnit.RADIANS, 0);
      odo.setPosition(startingPosition);
  }

  public void update() {
      odo.update();
      pos = odo.getPosition();
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
}
