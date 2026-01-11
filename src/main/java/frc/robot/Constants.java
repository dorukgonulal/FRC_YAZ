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

@SuppressWarnings("unused")

/**
 * Contains all the robot constants
 */
public final class Constants {
  public static final double stickDeadband = 0.1;
  // Kpler: autoalign 5-6ya çıkarılabilir., rotation pid 4-5 arasına
  // çıkarılabilir.
  // Zınk diye durmaması için ufak bir kD eklenebilir(translational için 0.1-0.2,
  // rotation için 0.1(Overshoot yoksa yapma varsa yap))
  // kEndTriggerDebounce'u 0.1 yaptım, eskiden 0.04'tü
  // rotation tolerance için 3 derece fazla olabilir daha da kısabiliriz.

  public static final class AutoConstants {
    public static final PIDConstants kRotationPID = new PIDConstants(1.5, 0, 0);// ok
    public static final Time kAutoAlignPredict = Seconds.of(0.0);

    // toleranslar
    public static final Rotation2d kRotationTolerance = Rotation2d.fromDegrees(5);
    public static final Distance kPositionTolerance = Centimeter.of(4);
    public static final LinearVelocity kSpeedTolerance = InchesPerSecond.of(2);

    public static final Time kEndTriggerDebounce = Seconds.of(0.1);

    public static final Time kTeleopAlignAdjustTimeout = Seconds.of(2);
    public static final Time kAutoAlignAdjustTimeout = Seconds.of(0.6);

    public static final PathConstraints kTeleopPathConstraints = new PathConstraints(2.5, 1.5,
        Units.degreesToRadians(540), Units.degreesToRadians(720)); // The constraints for this path.

    public static final PathConstraints kAutoPathConstraints = new PathConstraints(2.0, 1, 1 / 2 * Math.PI,
        1 * Math.PI); // ? consider making these more aggressive

    public static final PPHolonomicDriveController kAutoAlignPIDController = new PPHolonomicDriveController(
        new PIDConstants(0.8, 0, 0.1), // ok
        AutoConstants.kRotationPID);
  }
}
