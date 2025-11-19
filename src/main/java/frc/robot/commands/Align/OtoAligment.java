package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class OtoAligment extends Command {
  private final CommandSwerveDrivetrain drive;
  private final FieldPolygon fieldPolygon = new FieldPolygon();
  private final boolean left;

  public OtoAligment(CommandSwerveDrivetrain drive ,boolean left) {
    this.left = left;
    this.drive = drive;
    addRequirements(drive);
  }

  @Override
  public void initialize() {

    PathConstraints constraints =
        new PathConstraints(5, 5, Units.degreesToRadians(540), Units.degreesToRadians(720));

    int area = fieldPolygon.checkRobotPosition(drive.getState().Pose);

    Pose2d TargetPose = new Pose2d();
    if(left) {
      switch(area) {
        case 1:
        TargetPose = new Pose2d(5.152, 5.050, Rotation2d.fromDegrees(-120)); // SAĞ 5.313 5.225 -122 ||| SOL 5.313 5.050 -122
        break;
        case 2:
        TargetPose = new Pose2d(5.835, 3.889, Rotation2d.fromDegrees(180)); // SAĞ 5.835 3.889 180 ||| SOL 5.835 4.162 180
        break;
        case 3:
        TargetPose = new Pose2d(5.212, 2.994, Rotation2d.fromDegrees(120)); // SAĞ 5.212 2.851 120 ||| sol  5.212 2.994 120
        break;
        case 4:
        TargetPose = new Pose2d(3.846, 772, Rotation2d.fromDegrees(60)); // SAĞ 3.846 2.978 ||| SOL 3.84 2.772 60
        break;
        case 5:
        TargetPose = new Pose2d(3.148, 4.203, Rotation2d.fromDegrees(0)); // SAĞ 4.203 ||| 3.853
        break;
        case 6:
        TargetPose = new Pose2d(3.816, 5.205, Rotation2d.fromDegrees(-60));  // 5.062 ||| 5.205
        break;
      }
    } else if(!left) {
      switch(area) {
        case 1:
        TargetPose = new Pose2d(5.152, 5.225, Rotation2d.fromDegrees(-120)); // SAĞ 5.313 5.225 -122 ||| SOL 5.313 5.050 -122
        break;
        case 2:
        TargetPose = new Pose2d(5.835, 4.162, Rotation2d.fromDegrees(180)); // SAĞ 5.835 3.889 180 ||| SOL 5.835 4.162 180
        break;
        case 3:
        TargetPose = new Pose2d(5.212, 2.851, Rotation2d.fromDegrees(120)); // SAĞ 5.212 2.851 120 ||| sol  5.212 2.994 120
        break;
        case 4:
        TargetPose = new Pose2d(3.846, 2.978, Rotation2d.fromDegrees(60)); // SAĞ 3.846 2.978 ||| SOL 3.84 2.772 60
        break;
        case 5:
        TargetPose = new Pose2d(3.148, 3.853, Rotation2d.fromDegrees(0)); // SAĞ 4.203 ||| 3.853
        break;
        case 6:
        TargetPose = new Pose2d(3.816, 5.062, Rotation2d.fromDegrees(-60));  // 5.062 ||| 5.205
        break;
      }
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