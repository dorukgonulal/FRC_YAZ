package frc.robot.commands;

import org.opencv.core.RotatedRect;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;

public class PathFindingTest extends Command{
    Command pathfCommand;

    @Override
    public void initialize() {
        PathConstraints constraints =
        new PathConstraints(5, 5, Units.degreesToRadians(540), Units.degreesToRadians(720));

        AutoBuilder.pathfindToPose(new Pose2d(1.7, 3.2, new Rotation2d(0)), constraints).schedule();
        //pathfCommand = AutoBuilder.pathfindToPose(new Pose2d(1.7, 3.2, new Rotation2d(0)), constraints);
        //pathfCommand.schedule();

        //pathfCommand = AutoBuilder.pathfindToPose(new Pose2d(1.7, 3.2, new Rotation2d(0)), constraints);
        //pathfCommand.initialize();
    }

    @Override
    public void execute() {
        //pathfCommand.execute();
    }

    @Override
    public void end(boolean interrupted) {
        //pathFCommand.cancel();
    }

    @Override
    public boolean isFinished() {
        //pathFcommand.isFinished();
        return false;
    }
}
