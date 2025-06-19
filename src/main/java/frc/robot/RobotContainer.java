// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.FollowPathCommand;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.RobotID.Pivot.Coral;
import frc.robot.commands.climb.ClimbCommand;
import frc.robot.commands.climb.UnclimbCommand;
import frc.robot.commands.elevator.ElevatorDownCommand;
import frc.robot.commands.elevator.ElevatorUpCommand;
import frc.robot.commands.intake.AlgeaIntakeOnCommand;
import frc.robot.commands.intake.AlgeaShootCommand;
import frc.robot.commands.intake.CoralIntakeOnCommand;
import frc.robot.commands.intake.CoralShootCommand;
import frc.robot.commands.intake.CoralSlowShootCommand;
import frc.robot.commands.multi.AutoLifter;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.climb.ClimbSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem.ElevatorPositions;
import frc.robot.subsystems.intake.AlgeaSubsystem;
import frc.robot.subsystems.intake.CoralSubsystem;
import frc.robot.subsystems.pivot.AlgeaPivotSubsystem;
import frc.robot.subsystems.pivot.AlgeaPivotSubsystem.AlgeaPivotPosition;
import frc.robot.subsystems.pivot.CoralPivotSubsystem;
import frc.robot.subsystems.pivot.CoralPivotSubsystem.CoralPivotPosition;

public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second
                                                                                      // max angular velocity

  private final ClimbSubsystem climbSubsystem = new ClimbSubsystem();
  private final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();
  private final CoralPivotSubsystem coralPivotSubsystem = new CoralPivotSubsystem();
  private final CoralSubsystem coralSubsystem = new CoralSubsystem();
  private final AlgeaSubsystem algeaSubsystem = new AlgeaSubsystem();
  private final AlgeaPivotSubsystem algeaPivotSubsystem = new AlgeaPivotSubsystem();

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.2).withRotationalDeadband(MaxAngularRate * 0.2) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    private final SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController joystick = new CommandXboxController(0);

    private final Joystick operator = new Joystick(1);
    // private Joystick operator = new Joystick(1); // Joystick for manual control

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    /* Path follower */
    private final SendableChooser<Command> autoChooser;

    private final JoystickButton algeaIntake = new JoystickButton(operator, 5);
    private final JoystickButton algeaShoot = new JoystickButton(operator, 6);
    private final JoystickButton coralIntake = new JoystickButton(operator, 7);
    private final JoystickButton coralShoot = new JoystickButton(operator, 8);
    private final JoystickButton coralSlowShoot = new JoystickButton(operator, 14);
    private final JoystickButton hpfeed = new JoystickButton(operator, 9);
    private final JoystickButton elevatorUp = new JoystickButton(operator, 11);
    private final JoystickButton elevatorDown = new JoystickButton(operator, 12);

    private final JoystickButton coralDown = new JoystickButton(operator, 4);
    // private final JoystickButton coralDown = new JoystickButton(operator, 3);
    // private final JoystickButton algeaREEF = new JoystickButton(operator, 10);
    // private final JoystickButton algeaPROCESSOR = new JoystickButton(operator,
    // 13);
    // private final JoystickButton algeaUp = new JoystickButton(driver, 2);
    // private final JoystickButton algeaDown = new JoystickButton(driver, 3);

    private final JoystickButton algeaREEFA1 = new JoystickButton(operator, 1);
    private final JoystickButton algeaPROCESSOR = new JoystickButton(operator, 2);
    private final JoystickButton algeaREEFA2 = new JoystickButton(operator, 3);
    private final JoystickButton algeaGROUND = new JoystickButton(operator, 4);

    private final JoystickButton coralL1 = new JoystickButton(operator, 10);
    private final POVButton coralL2 = new POVButton(operator, 90);
    private final POVButton coralL3 = new POVButton(operator, 270);
    private final POVButton coralUp = new POVButton(operator, 0);
    private final POVButton coralBASE = new POVButton(operator, 180);

    public RobotContainer() {
        autoChooser = AutoBuilder.buildAutoChooser("Tests");
        SmartDashboard.putData("Auto Mode", autoChooser);
        SmartDashboard.putNumber("MAX SPEED", MaxSpeed);
        SmartDashboard.putNumber("MAX ANGULAR RATE", MaxAngularRate);
        configureBindings();

        NamedCommands.registerCommand("ElevatorHP", new AutoLifter(elevatorSubsystem, coralPivotSubsystem, algeaPivotSubsystem, ElevatorPositions.HP_FIRST, CoralPivotPosition.HP, AlgeaPivotPosition.CLOSE));
        NamedCommands.registerCommand("intake", new CoralIntakeOnCommand(coralSubsystem));
        NamedCommands.registerCommand("shoot", new CoralShootCommand(coralSubsystem));
        NamedCommands.registerCommand("ElevatorBASE", new AutoLifter(elevatorSubsystem, coralPivotSubsystem, algeaPivotSubsystem, ElevatorPositions.BASE, CoralPivotPosition.CLOSE, AlgeaPivotPosition.CLOSE));
        NamedCommands.registerCommand("ElevatorL4", new AutoLifter(elevatorSubsystem, coralPivotSubsystem, algeaPivotSubsystem, ElevatorPositions.L4, CoralPivotPosition.REEFL4, AlgeaPivotPosition.CLOSE));
        NamedCommands.registerCommand("ElevatorL3", new AutoLifter(elevatorSubsystem, coralPivotSubsystem, algeaPivotSubsystem, ElevatorPositions.L3, CoralPivotPosition.REEFL23, AlgeaPivotPosition.CLOSE));


        // Warmup PathPlanner to avoid Java pauses
        FollowPathCommand.warmupCommand().schedule();
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
                // Drivetrain will execute this command periodically
                drivetrain.applyRequest(() -> drive.withVelocityX(-joystick.getLeftY() * MaxSpeed) // Drive forward with
                                                                                                   // negative Y
                                                                                                   // (forward)
                        .withVelocityY(-joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                        .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with
                                                                                    // negative X (left)
                ));

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
                drivetrain.applyRequest(() -> idle).ignoringDisable(true));

        joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
        joystick.b().whileTrue(drivetrain.applyRequest(
                () -> point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))));

        joystick.pov(0).whileTrue(drivetrain.applyRequest(() -> forwardStraight.withVelocityX(0.5).withVelocityY(0)));
        joystick.pov(180)
                .whileTrue(drivetrain.applyRequest(() -> forwardStraight.withVelocityX(-0.5).withVelocityY(0)));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // reset the field-centric heading on left bumper press
        joystick.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        drivetrain.registerTelemetry(logger::telemeterize);

        // coralShoot.onTrue(new SequentialCommandGroup(
        // new CoralIntakeOnCommand(coralSubsystem, true), // Start intake
        // new WaitCommand(2),
        // new CoralIntakeOnCommand(coralSubsystem, false)
        // ));

        elevatorUp.whileTrue(new ElevatorUpCommand(elevatorSubsystem, 0.4));
        elevatorDown.whileTrue(new ElevatorDownCommand(elevatorSubsystem, 0.25));
        // elevatorBASE.onTrue(new CoralAutoLifter(elevatorSubsystem,
        // coralPivotSubsystem, algeaPivotSubsystem, ElevatorPositions.BASE,
        // CoralPivotPosition.CLOSE, AlgeaPivotPosition.CLOSE));
        // elevatorBASE.whileTrue(new ElevatorControlCommand(elevatorSubsystem,
        // ElevatorPositions.BASE));
        // elevatorl1.whileTrue(new ElevatorControlCommand(elevatorSubsystem,
        // ElevatorPositions.L1));

        coralIntake.whileTrue(new CoralIntakeOnCommand(coralSubsystem));
        coralShoot.whileTrue(new CoralShootCommand(coralSubsystem));
        coralSlowShoot.whileTrue(new CoralSlowShootCommand(coralSubsystem));

        algeaIntake.whileTrue(new AlgeaIntakeOnCommand(algeaSubsystem));
        algeaShoot.whileTrue(new AlgeaShootCommand(algeaSubsystem));

        // coralpivotdownbutton.onTrue(new CoralPivotControl(coralPivotSubsystem,
        // CoralPivotPosition.REEFL23));
        // coralpivotupbutton.onTrue(new CoralPivotControl(coralPivotSubsystem,
        // CoralPivotPosition.CLOSE));
        // algeapivotup.whileTrue(new AlgeaPivotUp(algeaPivotSubsystem));
        // algeapivotdown.whileTrue(new AlgeaPivotDown(algeaPivotSubsystem));

        algeaPROCESSOR.onTrue(new AutoLifter(elevatorSubsystem, coralPivotSubsystem, algeaPivotSubsystem,
                ElevatorPositions.PROCESSOR, CoralPivotPosition.CLOSE, AlgeaPivotPosition.PROCESSOR));
        algeaGROUND.onTrue(new AutoLifter(elevatorSubsystem, coralPivotSubsystem, algeaPivotSubsystem,
                ElevatorPositions.L1, CoralPivotPosition.CLOSE, AlgeaPivotPosition.GROUND));
        // coralUp.whileTrue(new CoralPivotUp(coralPivotSubsystem));
        // coralDown.whileTrue(new CoralPivotDown(coralPivotSubsystem));

        // algeaREEF.onTrue(new AlgeaPivotControl(algeaPivotSubsystem,
        // AlgeaPivotPosition.REEF));
        // algeaPROCESSOR.onTrue(new AlgeaPivotControl(algeaPivotSubsystem,
        // AlgeaPivotPosition.PROCESSOR));

        // coralpivotbase.onTrue(new CoralPivotControl(coralPivotSubsystem,
        // CoralPivotPosition.CLOSE));
        // coralpivotnormal.onTrue(new CoralPivotControl(coralPivotSubsystem,
        // CoralPivotPosition.NORMAL));
        // coralUp.whileTrue(new CoralPivotUp(coralPivotSubsystem));
        // coralDown.whileTrue(new CoralPivotDown(coralPivotSubsystem));
        // algeapivotbase.onTrue(new AlgeaPivotControl(algeaPivotSubsystem,
        // AlgeaPivotPosition.CLOSE));
        // algeapivotnormal.onTrue(new AlgeaPivotControl(algeaPivotSubsystem,
        // AlgeaPivotPosition.REEF));
        coralBASE.onTrue(
                new AutoLifter(elevatorSubsystem, coralPivotSubsystem, algeaPivotSubsystem, ElevatorPositions.BASE,
                        CoralPivotPosition.CLOSE, AlgeaPivotPosition.CLOSE));
        coralL1.onTrue(
                new AutoLifter(elevatorSubsystem, coralPivotSubsystem, algeaPivotSubsystem, ElevatorPositions.L1,
                        CoralPivotPosition.REEFL1, AlgeaPivotPosition.CLOSE));
        coralL2.onTrue(
                new AutoLifter(elevatorSubsystem, coralPivotSubsystem, algeaPivotSubsystem, ElevatorPositions.L2,
                        CoralPivotPosition.REEFL23, AlgeaPivotPosition.CLOSE));
        coralL3.onTrue(
                new AutoLifter(elevatorSubsystem, coralPivotSubsystem, algeaPivotSubsystem, ElevatorPositions.L3,
                        CoralPivotPosition.REEFL23, AlgeaPivotPosition.CLOSE));
        // coralL4.onTrue(
        // new AutoLifter(elevatorSubsystem, coralPivotSubsystem, algeaPivotSubsystem,
        // ElevatorPositions.L4,
        // CoralPivotPosition.REEFL4, AlgeaPivotPosition.CLOSE));

        hpfeed.onTrue(
                new SequentialCommandGroup(
                        new AutoLifter(elevatorSubsystem, coralPivotSubsystem, algeaPivotSubsystem,
                                ElevatorPositions.HP_FIRST,
                                CoralPivotPosition.HP, AlgeaPivotPosition.CLOSE),
                        new WaitCommand(0.5),
                        new AutoLifter(elevatorSubsystem, coralPivotSubsystem, algeaPivotSubsystem,
                                ElevatorPositions.HP_LAST, CoralPivotPosition.HP, AlgeaPivotPosition.CLOSE)));

    }

    public Command getAutonomousCommand() {
        /* Run the path selected from the auto chooser */
        return autoChooser.getSelected();
    }
}
