package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.util.FieldPolygon;

public class OtoAligment extends Command {
  private final CommandSwerveDrivetrain drive;
  private final FieldPolygon fieldPolygon = new FieldPolygon();
  private final boolean left;

  public OtoAligment(CommandSwerveDrivetrain drive, boolean left) {
    this.drive = drive;
    this.left = left;
    addRequirements(drive);
  }

  @Override
  public void initialize() {

    PathConstraints constraints =
        new PathConstraints(3.0, 3.0, Units.degreesToRadians(540), Units.degreesToRadians(720));

    int area = fieldPolygon.checkRobotPosition(drive.getState().Pose);

    Pose2d TargetPose = new Pose2d();
    if (area == 1) {
      TargetPose = new Pose2d(5.152, 5.183, Rotation2d.fromDegrees(-120));
    } else if (area == 2) {
      TargetPose = new Pose2d(5.835, 3.966, Rotation2d.fromDegrees(180));
    } else if (area == 3) {
      TargetPose = new Pose2d(5.212, 2.882, Rotation2d.fromDegrees(120));
    } else if (area == 4) {
      TargetPose = new Pose2d(3.846, 2.882, Rotation2d.fromDegrees(60));
    } else if (area == 5) {
      TargetPose = new Pose2d(3.148, 4.025, Rotation2d.fromDegrees(0));
    } else if (area == 6) {
      TargetPose = new Pose2d(3.816, 5.168, Rotation2d.fromDegrees(-60));
    }

    AutoBuilder.pathfindToPose(TargetPose, constraints).schedule();
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return true;
  }
}