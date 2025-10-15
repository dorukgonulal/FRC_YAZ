package frc.robot;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;

import static edu.wpi.first.units.Units.Centimeter;
import static edu.wpi.first.units.Units.InchesPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import java.io.IOException;

/**
 * Contains all the robot constants
 */
public final class Constants {
    public static final double stickDeadband = 0.1;

    public final class AprilConstants {
      public static final double X_REEF_ALIGNMENT_P = 1.5;
      public static final double Y_REEF_ALIGNMENT_P = 2.4;
      public static final double ROT_REEF_ALIGNMENT_P = 0.04;
      public static final double ROT_SETPOINT_REEF_ALIGNMENT = 0;
      public static final double ROT_TOLERANCE_REEF_ALIGNMENT = 0;
      public static final double X_SETPOINT_REEF_ALIGNMENT = -0.5;
      public static final double Y_SETPOINT_REEF_ALIGNMENT = 0.15;
      public static final double X_TOLERANCE_REEF_ALIGNMENT = 0.05;
      public static final double Y_TOLERANCE_REEF_ALIGNMENT = 0.12;
      public static final double DONT_SEE_TAG_WAIT_TIME = 1;
      public static final double POSE_VALIDATION_TIME = 0.3;
      
  
    }

    public static class ElevatorConstants {
      public static final double kEncoderTick2Meter = 0;
      public static final double ELEVATOR_TOLERANCE = 0.05;
      public static final double ELEVATOR_KP = 0.1;
      public static final double ELEVATOR_POWER = 0.4;
      public static final double ELEVATOR_SETPOINT_BASE = -0.1;

      public static final double ELEVATOR_SETPOINT_L1 = 10;
      public static final double ELEVATOR_SETPOINT_L2 = 38.2;
      public static final double ELEVATOR_AUTO_L2 = 39.7;

      public static final double ELEVATOR_SETPOINT_READYFORAUTO = 24;

      public static final double ELEVATOR_SETPOINT_L3 = 44;
      public static final double ELEVATOR_SETPOINT_L4 = 69.5;

      public static final double ELEVATOR_SETPOINT_AUTOL3 = 67.3;

      public static final double ELEVATOR_SETPOINT_ALGEA1 = 26.2;
      public static final double ELEVATOR_SETPOINT_ALGEA2 = 41.2;
      public static final double ELEVATOR_SETPOINT_PROCESSOR = 18;

      public static final double ELEVATOR_SETPOINT_HP = 20;
      public static final double ELEVATOR_SETPOINT_MAKS = 12.5;
    }

    public static final class IntakeConstants {
      public static final double IntakePower = 0.35;
  }

  public static final class AutoConstants {
    public static final PIDConstants kRotationPID = new PIDConstants(22, 0, 0.05);//ok
    public static final Time kAutoAlignPredict = Seconds.of(0.0);


    //toleranslar
    public static final Rotation2d kRotationTolerance = Rotation2d.fromDegrees(3.0);
    public static final Distance kPositionTolerance = Centimeter.of(1.5);
    public static final LinearVelocity kSpeedTolerance = InchesPerSecond.of(2);

    public static final Time kEndTriggerDebounce = Seconds.of(0.04);

    public static final Time kTeleopAlignAdjustTimeout = Seconds.of(2);
    public static final Time kAutoAlignAdjustTimeout = Seconds.of(0.6);

    public static final PathConstraints kTeleopPathConstraints = new PathConstraints(2.5, 2.0, 1/2 * Math.PI, 1 * Math.PI); // The constraints for this path.

    public static final PathConstraints kAutoPathConstraints = new PathConstraints(2.25, 2.25, 1/2 * Math.PI, 1 * Math.PI); //? consider making these more aggressive
    
    public static final PPHolonomicDriveController kAutoAlignPIDController = new PPHolonomicDriveController(
      new PIDConstants(9, 0, 0.25), //ok
        AutoConstants.kRotationPID
    );}

  public static final class PivotConstants {
    public static final double PIVOT_KP = 0.07;
    public static final double PIVOT_KI = 0.0002;
    public static final double PIVOT_KD = 0.0002;

    public static final double CLOSE = 0.0;
    public static final double NORMAL = -5;
    public static final double HP = -10.7;
    public static final double REEFL1 = -9;
    public static final double REEF_L2_L3 = -25.6;
    public static final double REEF_L4 = -27.2;
    public static final double BARGE = -10;
    public static final double PROCESSOR = -14;
    public static final double A1 = -22.5;
    public static final double A2 = -22.4;

    public static final double PIVOT_POWER = 0.65;
  }  
}