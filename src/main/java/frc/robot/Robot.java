// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Degrees;

import com.ctre.phoenix6.configs.CANdleConfiguration;
import com.ctre.phoenix6.controls.ColorFlowAnimation;
import com.ctre.phoenix6.controls.EmptyAnimation;
import com.ctre.phoenix6.controls.FireAnimation;
import com.ctre.phoenix6.controls.LarsonAnimation;
import com.ctre.phoenix6.controls.RainbowAnimation;
import com.ctre.phoenix6.controls.RgbFadeAnimation;
import com.ctre.phoenix6.controls.SingleFadeAnimation;
import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.controls.StrobeAnimation;
import com.ctre.phoenix6.controls.TwinkleAnimation;
import com.ctre.phoenix6.controls.TwinkleOffAnimation;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.AnimationDirectionValue;
import com.ctre.phoenix6.signals.RGBWColor;
import com.ctre.phoenix6.signals.StatusLedWhenActiveValue;
import com.ctre.phoenix6.signals.StripTypeValue;
import com.pathplanner.lib.pathfinding.LocalADStar;
import com.pathplanner.lib.pathfinding.Pathfinding;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {
  /* color can be constructed from RGBW, a WPILib Color/Color8Bit, HSV, or hex */
  private static final RGBWColor kGreen = new RGBWColor(0, 217, 0, 0);
  private static final RGBWColor kWhite = new RGBWColor(Color.kWhite).scaleBrightness(0.5);
  private static final RGBWColor kViolet = RGBWColor.fromHSV(Degrees.of(270), 0.9, 0.8);
  private static final RGBWColor kRed = RGBWColor.fromHex("#D9000000").orElseThrow();

  /*
   * Start and end index for LED animations.
   * 0-7 are onboard, 8-399 are an external strip.
   * CANdle supports 8 animation slots (0-7).
   */
  private static final int kSlot0StartIdx = 0;
  private static final int kSlot0EndIdx = 7;

  private static final int kSlot1StartIdx = 38;
  private static final int kSlot1EndIdx = 67;

  private final CANdle m_candle = new CANdle(44, "CANivore");

  private enum AnimationType {
    None,
    ColorFlow,
    Fire,
    Larson,
    Rainbow,
    RgbFade,
    SingleFade,
    Strobe,
    Twinkle,
    TwinkleOff,
  }

  private AnimationType m_anim0State = AnimationType.None;
  private AnimationType m_anim1State = AnimationType.None;

  private final SendableChooser<AnimationType> m_anim0Chooser = new SendableChooser<AnimationType>();
  private final SendableChooser<AnimationType> m_anim1Chooser = new SendableChooser<AnimationType>();
  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;

  private final boolean kUseLimelight = false;

  public Robot() {
    /* Configure CANdle */
    var cfg = new CANdleConfiguration();
    /* set the LED strip type and brightness */
    cfg.LED.StripType = StripTypeValue.GRB;
    cfg.LED.BrightnessScalar = 0.5;
    /* disable status LED when being controlled */
    cfg.CANdleFeatures.StatusLedWhenActive = StatusLedWhenActiveValue.Disabled;

    m_candle.getConfigurator().apply(cfg);

    /* clear all previous animations */
    for (int i = 0; i < 8; ++i) {
      m_candle.setControl(new EmptyAnimation(i));
    }
    /* set the onboard LEDs to a solid color */
    m_candle.setControl(new SolidColor(0, 3).withColor(kGreen));
    m_candle.setControl(new SolidColor(4, 7).withColor(kWhite));

    /* add animations to chooser for slot 0 */
    m_anim0Chooser.setDefaultOption("Color Flow", AnimationType.ColorFlow);
    m_anim0Chooser.addOption("Rainbow", AnimationType.Rainbow);
    m_anim0Chooser.addOption("Twinkle", AnimationType.Twinkle);
    m_anim0Chooser.addOption("Twinkle Off", AnimationType.TwinkleOff);
    m_anim0Chooser.addOption("Fire", AnimationType.Fire);

    /* add animations to chooser for slot 1 */
    m_anim1Chooser.setDefaultOption("Larson", AnimationType.Larson);
    m_anim1Chooser.addOption("RGB Fade", AnimationType.RgbFade);
    m_anim1Chooser.addOption("Single Fade", AnimationType.SingleFade);
    m_anim1Chooser.addOption("Strobe", AnimationType.Strobe);
    m_anim1Chooser.addOption("Fire", AnimationType.Fire);

    SmartDashboard.putData("Animation 0", m_anim0Chooser);
    SmartDashboard.putData("Animation 1", m_anim1Chooser);
    m_robotContainer = new RobotContainer();
    Pathfinding.setPathfinder(new LocalADStar());
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    /* if the selection for slot 0 changes, change animations */
    final var anim0Selection = m_anim0Chooser.getSelected();
    if (m_anim0State != anim0Selection) {
      m_anim0State = anim0Selection;

      switch (m_anim0State) {
        default:
        case ColorFlow:
          m_candle.setControl(
              new ColorFlowAnimation(kSlot0StartIdx, kSlot0EndIdx).withSlot(0)
                  .withColor(kViolet));
          break;
        case Rainbow:
          m_candle.setControl(
              new RainbowAnimation(kSlot0StartIdx, kSlot0EndIdx).withSlot(0));
          break;
        case Twinkle:
          m_candle.setControl(
              new TwinkleAnimation(kSlot0StartIdx, kSlot0EndIdx).withSlot(0)
                  .withColor(kViolet));
          break;
        case TwinkleOff:
          m_candle.setControl(
              new TwinkleOffAnimation(kSlot0StartIdx, kSlot0EndIdx).withSlot(0)
                  .withColor(kViolet));
          break;
        case Fire:
          m_candle.setControl(
              new FireAnimation(kSlot0StartIdx, kSlot0EndIdx).withSlot(0));
          break;
      }
    }

    /* if the selection for slot 1 changes, change animations */
    final var anim1Selection = m_anim1Chooser.getSelected();
    if (m_anim1State != anim1Selection) {
      m_anim1State = anim1Selection;

      switch (m_anim1State) {
        default:
        case Larson:
          m_candle.setControl(
              new LarsonAnimation(kSlot1StartIdx, kSlot1EndIdx).withSlot(1)
                  .withColor(kRed));
          break;
        case RgbFade:
          m_candle.setControl(
              new RgbFadeAnimation(kSlot1StartIdx, kSlot1EndIdx).withSlot(1));
          break;
        case SingleFade:
          m_candle.setControl(
              new SingleFadeAnimation(kSlot1StartIdx, kSlot1EndIdx).withSlot(1)
                  .withColor(kRed));
          break;
        case Strobe:
          m_candle.setControl(
              new StrobeAnimation(kSlot1StartIdx, kSlot1EndIdx).withSlot(1)
                  .withColor(kRed));
          break;
        case Fire:
          /*
           * direction can be reversed by either the Direction parameter or switching
           * start and end
           */
          m_candle.setControl(
              new FireAnimation(kSlot1StartIdx, kSlot1EndIdx).withSlot(1)
                  .withDirection(AnimationDirectionValue.Backward)
                  .withCooling(0.4)
                  .withSparking(0.5));
          break;
      }
    }
    /*
     * This example of adding Limelight is very simple and may not be sufficient for
     * on-field use.
     * Users typically need to provide a standard deviation that scales with the
     * distance to target
     * and changes with number of tags available.
     *
     * This example is sufficient to show that vision integration is possible,
     * though exact implementation
     * of how to use vision should be tuned per-robot and to the team's
     * specification.
     */
    if (kUseLimelight) {
      var driveState = m_robotContainer.drivetrain.getState();
      double headingDeg = driveState.Pose.getRotation().getDegrees();
      double omegaRps = Units.radiansToRotations(driveState.Speeds.omegaRadiansPerSecond);

      LimelightHelpers.SetRobotOrientation("limelight", headingDeg, 0, 0, 0, 0, 0);
      var llMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight");
      if (llMeasurement != null && llMeasurement.tagCount > 0 && Math.abs(omegaRps) < 2.0) {
        m_robotContainer.drivetrain.addVisionMeasurement(llMeasurement.pose, llMeasurement.timestampSeconds);
      }
    }
  }

  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
  }

  @Override
  public void disabledExit() {
  }

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void autonomousExit() {
  }

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void teleopExit() {
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {
  }

  @Override
  public void testExit() {
  }

  @Override
  public void simulationPeriodic() {
  }
}
