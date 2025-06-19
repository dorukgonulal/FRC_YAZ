package frc.robot;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.pathplanner.lib.config.PIDConstants;

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
import edu.wpi.first.wpilibj.simulation.ElevatorSim;

import java.io.IOException;

/**
 * Contains all the robot constants
 */
public final class Constants {
    public static final double stickDeadband = 0.1;

    public static class ElevatorConstants {
      public static final double kEncoderTick2Meter = 0;
      public static final double ELEVATOR_TOLERANCE = 0.05;
      public static final double ELEVATOR_KP = 0.1;
      public static final double ELEVATOR_POWER = 0.4;
      public static final double ELEVATOR_SETPOINT_BASE = -0.1;

      public static final double ELEVATOR_SETPOINT_L1 = 10;
      public static final double ELEVATOR_SETPOINT_L2 = 39.7;
      public static final double ELEVATOR_AUTO_L2 = 39.7;

      public static final double ELEVATOR_SETPOINT_L3 = 63.4;
      public static final double ELEVATOR_SETPOINT_L4 = 72;

      public static final double ELEVATOR_SETPOINT_AUTOL3 = 67.3;

      public static final double ELEVATOR_SETPOINT_ALGEA1 = 45.5;
      public static final double ELEVATOR_SETPOINT_ALGEA2 = 67.3;
      public static final double ELEVATOR_SETPOINT_PROCESSOR = 18;

      public static final double ELEVATOR_SETPOINT_HP_FIRST = 29;
      public static final double ELEVATOR_SETPOINT_HP_LAST = ELEVATOR_SETPOINT_HP_FIRST - 2.0;
      public static final double ELEVATOR_SETPOINT_MAKS = 12.5;
    }

    public static final class IntakeConstants {
      public static final class CoralConstants {
          public static final double IntakePower = 0.35;
      }
  }

  public static final class PivotConstants {
      public static final class AlgeaPivot {
        public static final double PIVOT_KP = 0.05;
        public static final double PIVOT_KI = 0.0002;
        public static final double PIVOT_KD = 0.0002;
    
        public static final double CLOSE = 0.7;
        public static final double GROUND = -27.2;
        public static final double REEFA1 = -25;
        public static final double REEFA2 = -23.3;
        public static final double PROCESSOR = -19;
        public static final double PIVOT_POWER = 0.65;
      }

      public static final class CoralPivot {
        public static final double PIVOT_KP = 0.07;
        public static final double PIVOT_KI = 0.0002;
        public static final double PIVOT_KD = 0.0002;
    
        public static final double CLOSE_CORAL = 0.0;
        public static final double NORMAL = -5;
        public static final double HP_CORAL = -10.7;
        public static final double REEFL1 = -9;
        public static final double REEF_L2_L3_CORAL = -29;
          public static final double REEF_L4_CORAL = -27;
    
        public static final double PIVOT_POWER = 0.65;
      }
  }  
}