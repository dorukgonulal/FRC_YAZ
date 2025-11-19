package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class AutoHP extends Command {

    private final Pose2d hpPose1 = new Pose2d(1.082, 1.103, Rotation2d.fromDegrees(-120)); //Sallama değerler
    private final Pose2d hpPose2 = new Pose2d(1.082, 7, Rotation2d.fromDegrees(120));    //Sallama değerler
    private final CommandSwerveDrivetrain drivetrain;
    private Command pathfindCommand;

    public AutoHP(CommandSwerveDrivetrain drivetrain) {
        this.drivetrain = drivetrain;
    }

    @Override
    public void initialize() {
        PathConstraints constraints =
                new PathConstraints(5, 5, Units.degreesToRadians(540), Units.degreesToRadians(720));

        // Robotun o anki pozisyonuna göre hedef belirle
        pathfindCommand = AutoBuilder.pathfindToPose(isHp1() ? hpPose1 : hpPose2, constraints);
        pathfindCommand.initialize();
    }

    @Override
    public void execute() {
        if (pathfindCommand != null) {
            pathfindCommand.execute();
        }
    }

    @Override
    public void end(boolean interrupted) {
        if (pathfindCommand != null) {
            pathfindCommand.end(interrupted);
        }
    }

    @Override
    public boolean isFinished() {
        return pathfindCommand != null && pathfindCommand.isFinished();
    }

    // Eğer Y koordinatı <4 ise hpPose1, >=4 ise hpPose2
    public boolean isHp1() {
        return drivetrain.getState().Pose.getY() < 4;
    }
}
