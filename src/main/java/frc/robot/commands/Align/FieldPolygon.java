package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;

public class FieldPolygon {

  //Method to control which area the robot is in
  public int checkRobotPosition(Pose2d RobotPose) {
    double x = RobotPose.getX();
    double y = RobotPose.getY();

    if (x >= 4.501 && x <= 8.768 && y >= 6.176 && y <= 8.000) {
      // Yellow
      return 1;
    } else if (x >= 4.501 && x <= 8.768 && y >= 1.902 && y <= 6.176) {
      // Green
      return 2;
    } else if (x >= 4.501 && x <= 8.768 && y >= 0 && y <= 1.902) {
      // Blue
      return 3;
    } else if (x >= 0 && x <= 4.501 && y >= 0 && y <= 1.240) {
      // Pink
      return 4;
    } else if (x >= 0 && x <= 4.501 && y >= 1.240 && y <= 6.760) {
      // Red
      return 5;
    } else if (x >= 0 && x <= 4.501 && y >= 6.760 && y <= 8.000) {
      // Orange
      return 6;
    } else {
      // no field in
      return 0;
    }
  }
}