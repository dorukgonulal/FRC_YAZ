// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class AlignToReefTagRelative extends Command {
    private PIDController xController, yController, rotController;
    private boolean isRightScore;
    private Timer dontSeeTagTimer, stopTimer;
    private CommandSwerveDrivetrain drivebase;
    private SwerveRequest.RobotCentric m_drive = new SwerveRequest.RobotCentric();
    private double tagID = -1;

    public AlignToReefTagRelative(boolean isRightScore, CommandSwerveDrivetrain drivebase) {
        xController = new PIDController(Constants.AprilConstants.X_REEF_ALIGNMENT_P, 0.0, 0); // Vertical movement
        yController = new PIDController(Constants.AprilConstants.Y_REEF_ALIGNMENT_P, 0.0, 0); // Horitontal movement
        rotController = new PIDController(Constants.AprilConstants.ROT_REEF_ALIGNMENT_P, 0, 0); // Rotation
        this.isRightScore = isRightScore;
        this.drivebase = drivebase;
        addRequirements(drivebase);
    }

    @Override
    public void initialize() {
        this.stopTimer = new Timer();
        this.stopTimer.start();
        this.dontSeeTagTimer = new Timer();
        this.dontSeeTagTimer.start();

        rotController.setSetpoint(Constants.AprilConstants.ROT_SETPOINT_REEF_ALIGNMENT);
        rotController.setTolerance(Constants.AprilConstants.ROT_TOLERANCE_REEF_ALIGNMENT);

        xController.setSetpoint(Constants.AprilConstants.X_SETPOINT_REEF_ALIGNMENT);
        xController.setTolerance(Constants.AprilConstants.X_TOLERANCE_REEF_ALIGNMENT);

        yController.setSetpoint(isRightScore ? Constants.AprilConstants.Y_SETPOINT_REEF_ALIGNMENT
                : -Constants.AprilConstants.Y_SETPOINT_REEF_ALIGNMENT - 0.1);
        yController.setTolerance(Constants.AprilConstants.Y_TOLERANCE_REEF_ALIGNMENT);

        tagID = LimelightHelpers.getFiducialID("limelight-reef");
    }

    @Override
    public void execute() {
        if (LimelightHelpers.getTV("limelight-reef") && LimelightHelpers.getFiducialID("limelight-reef") == tagID) {
            this.dontSeeTagTimer.reset();

            double[] postions = LimelightHelpers.getBotPose_TargetSpace("limelight-reef");
            // SmartDashboard.putNumber("x", postions[2]);

            double xSpeed = xController.calculate(postions[2]);
            // SmartDashboard.putNumber("xspee", xSpeed);
            double ySpeed = -yController.calculate(postions[0]);
            double rotValue = -rotController.calculate(postions[4]);

            drivebase.setControl(m_drive
                    .withVelocityX(xSpeed)
                    .withVelocityY(ySpeed)
                    .withRotationalRate(rotValue));

            if (!rotController.atSetpoint() ||
                    !yController.atSetpoint() ||
                    !xController.atSetpoint()) {
                stopTimer.reset();
                ySpeed = Math.max(ySpeed, 0.15);
                xSpeed = Math.max(xSpeed, 0.23);
            }
        } else {
            drivebase.setControl(m_drive
                    .withVelocityX(0)
                    .withVelocityY(0)
                    .withRotationalRate(0));
        }

        // SmartDashboard.putNumber("poseValidTimer", stopTimer.get());
    }

    @Override
    public void end(boolean interrupted) {
        drivebase.setControl(m_drive
                .withVelocityX(0)
                .withVelocityY(0)
                .withRotationalRate(0));
    }

    @Override
    public boolean isFinished() {
        // Requires the robot to stay in the correct position for 0.3 seconds, as long
        // as it gets a tag in the camera
        // return
        // this.dontSeeTagTimer.hasElapsed(Constants.AprilConstants.DONT_SEE_TAG_WAIT_TIME)
        // ||
        // stopTimer.hasElapsed(Constants.AprilConstants.POSE_VALIDATION_TIME);

        if ((Math.abs(LimelightHelpers.getTX("limelight-reef")) < 5 && LimelightHelpers.getTY("limelight-reef") > 2)
                || !LimelightHelpers.getTV("limelight-reef")) {
            return true;
        } else {
            return false;
        }
    }
}