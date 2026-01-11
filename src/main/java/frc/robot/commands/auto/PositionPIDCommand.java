package frc.robot.commands.auto;

import static edu.wpi.first.units.Units.Centimeter;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.trajectory.PathPlannerTrajectoryState;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;

@SuppressWarnings("unused")

public class PositionPIDCommand extends Command {

    public CommandSwerveDrivetrain mSwerve;
    public final Pose2d goalPose;
    private PPHolonomicDriveController mDriveController = AutoConstants.kAutoAlignPIDController;
    private static final Time kEndTriggerDebounce = AutoConstants.kEndTriggerDebounce;
    private static final Distance kPositionTolerance = AutoConstants.kPositionTolerance;
    private static final LinearVelocity kSpeedTolerance = AutoConstants.kSpeedTolerance;
    public static final Rotation2d kRotationTolerance = AutoConstants.kRotationTolerance;

    private final Timer timer = new Timer();

    private final Debouncer endTriggerDebouncer = new Debouncer(kEndTriggerDebounce.in(Seconds));

    private PositionPIDCommand(CommandSwerveDrivetrain mSwerve, Pose2d goalPose) {
        this.mSwerve = mSwerve;
        this.goalPose = goalPose;
    }
    
    public static Command generateCommand(CommandSwerveDrivetrain swerve, Pose2d goalPose, Time timeout) {
        return new PositionPIDCommand(swerve, goalPose).withTimeout(timeout).finallyDo(() -> {
            swerve.setControl(
                    new SwerveRequest.FieldCentric()
                            .withVelocityX(0)
                            .withVelocityY(0)
                            .withRotationalRate(0));
            swerve.setControl(new SwerveRequest.SwerveDriveBrake());
        });
    }

    @Override
    public void initialize() {
        timer.restart();
    }

    @Override
    public void execute() {

        PathPlannerTrajectoryState goalState = new PathPlannerTrajectoryState();
        goalState.pose = goalPose;

        // System.out.println("PİD ÇALIŞIYORR");
        var robotSpeeds = mDriveController.calculateRobotRelativeSpeeds(mSwerve.getPose(), goalState);

        // DENE
        if (Math.abs(robotSpeeds.vxMetersPerSecond) < 0.05 && Math.abs(robotSpeeds.vyMetersPerSecond) < 0.05) {
            mSwerve.setControl(new SwerveRequest.SwerveDriveBrake());
            return;
        }

        mSwerve.setControl(new SwerveRequest.ApplyRobotSpeeds().withSpeeds(robotSpeeds));

    }

    @Override
    public void end(boolean interrupted) {
        timer.stop();

        Pose2d diff = mSwerve.getPose().relativeTo(goalPose);

        System.out.println("Adjustments to alginment took: " + timer.get() + " seconds and interrupted = " + interrupted
                + "\nPosition offset: " + Centimeter.convertFrom(diff.getTranslation().getNorm(), Meters) + " cm"
                + "\nRotation offset: " + diff.getRotation().getMeasure().in(Degrees) + " deg"
                + "\nVelocity value: " + mSwerve.getSpeed() + "m/s");
    }

    @Override
    public boolean isFinished() {

        Pose2d diff = mSwerve.getPose().relativeTo(goalPose);

        var rotation = MathUtil.isNear(
                0.0,
                diff.getRotation().getRotations(),
                kRotationTolerance.getRotations(),
                0.0,
                1.0);

        var position = diff.getTranslation().getNorm() < kPositionTolerance.in(Meters);

        double linespeed = AlignToReef.getVelocityMagnitude(mSwerve.getSpeed()).in(MetersPerSecond);

        var speed = linespeed < kSpeedTolerance.in(MetersPerSecond);

        // System.out.println("end trigger conditions R: "+ rotation + "\tP: " +
        // position + "\tS: " + speed);

        return endTriggerDebouncer.calculate(
                rotation && position && speed);
    }
}