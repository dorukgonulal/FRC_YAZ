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
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.pathfinding.Pathfinding;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.RobotID.Pivot;
import frc.robot.commands.AlignToReefTagRelative;
import frc.robot.commands.GoToPath;
import frc.robot.commands.climb.ClimbCommand;
import frc.robot.commands.climb.UnclimbCommand;
import frc.robot.commands.elevator.ElevatorDownCommand;
import frc.robot.commands.elevator.ElevatorUpCommand;
import frc.robot.commands.intake.IntakeOnCommand;
import frc.robot.commands.intake.IntakeShootCommand;
import frc.robot.commands.multi.AutoLifter;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.climb.ClimbSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.pivot.PivotSubsystem;
import frc.robot.subsystems.pivot.PivotSubsystem.PivotPosition;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem.ElevatorPositions;
import edu.wpi.first.wpilibj2.command.CommandScheduler;


public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top
                                                                                  // speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per
                                                                                      // second
                                                                                      // max angular velocity

    private final ClimbSubsystem climbSubsystem = new ClimbSubsystem();
    private final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();
    private final PivotSubsystem pivotSubsystem = new PivotSubsystem();
    private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.12).withRotationalDeadband(MaxAngularRate * 0.12) // Add a 10%
                                                                                         // deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive
                                                                     // motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    private final SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController joystick = new CommandXboxController(0);

    private final Joystick operator = new Joystick(1);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    public final AutoLifter autoLifter = new AutoLifter(elevatorSubsystem, pivotSubsystem,
            ElevatorPositions.BASE, PivotPosition.CLOSE);

    /* Path follower */
    private final SendableChooser<Command> autoChooser;

    private final JoystickButton intake = new JoystickButton(operator, 7);
    private final JoystickButton shoot = new JoystickButton(operator, 8);

    private final JoystickButton coralSlowShoot = new JoystickButton(operator, 14);

    private final JoystickButton hpfeed = new JoystickButton(operator, 9);
    private final JoystickButton elevatorUp = new JoystickButton(operator, 11);
    private final JoystickButton elevatorDown = new JoystickButton(operator, 12);

    private final JoystickButton algeaREEFA1 = new JoystickButton(operator, 1);
    private final JoystickButton algeaPROCESSOR = new JoystickButton(operator, 2);
    private final JoystickButton algeaREEFA2 = new JoystickButton(operator, 3);
    private final JoystickButton algeaGROUND = new JoystickButton(operator, 4);

    private final JoystickButton coralL1 = new JoystickButton(operator, 10);
    private final POVButton coralL2 = new POVButton(operator, 90);
    private final POVButton coralL3 = new POVButton(operator, 270);
    private final POVButton coralL4 = new POVButton(operator, 0);
    private final POVButton coralBASE = new POVButton(operator, 180);

    public RobotContainer() {
        SmartDashboard.putNumber("MAX SPEED", MaxSpeed);
        SmartDashboard.putNumber("MAX ANGULAR RATE", MaxAngularRate);

        NamedCommands.registerCommand("ElevatorHP",
                new AutoLifter(elevatorSubsystem, pivotSubsystem,
                        ElevatorPositions.HP, PivotPosition.HP));
        NamedCommands.registerCommand("intake", new IntakeOnCommand(intakeSubsystem));
        NamedCommands.registerCommand("shoot", new IntakeShootCommand(intakeSubsystem));
        NamedCommands.registerCommand("ElevatorBASE",
                new AutoLifter(elevatorSubsystem, pivotSubsystem, ElevatorPositions.BASE, PivotPosition.CLOSE));
        NamedCommands.registerCommand("ElevatorL4",
                new AutoLifter(elevatorSubsystem, pivotSubsystem, ElevatorPositions.L4, PivotPosition.REEFL4));
        NamedCommands.registerCommand("ElevatorL3",
                new AutoLifter(elevatorSubsystem, pivotSubsystem, ElevatorPositions.L3, PivotPosition.REEFL23));

        // Warmup PathPlanner to avoid Java pauses
        FollowPathCommand.warmupCommand().schedule();
        PathfindingCommand.warmupCommand().schedule();
        autoChooser = AutoBuilder.buildAutoChooser("Tests");
        SmartDashboard.putData("Auto Mode", autoChooser);
        configureBindings();
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
                // Drivetrain will execute this command periodically
                drivetrain.applyRequest(() -> drive.withVelocityX(-joystick.getLeftY() * MaxSpeed) // Drive
                                                                                                   // forward
                                                                                                   // with
                                                                                                   // negative
                                                                                                   // Y
                                                                                                   // (forward)
                        .withVelocityY(-joystick.getLeftX() * MaxSpeed) // Drive left with
                                                                        // negative X (left)
                        .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive
                                                                                    // counterclockwise
                                                                                    // with
                                                                                    // negative
                                                                                    // X (left)
                ));

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
                drivetrain.applyRequest(() -> idle).ignoringDisable(true));

        joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
        joystick.b().onTrue(new AlignToReefTagRelative(true, drivetrain));
        joystick.x().onTrue(new AlignToReefTagRelative(false, drivetrain));

        // joystick.b().onTrue(new SequentialCommandGroup(
        //         new AlignToReefTagRelative(true, drivetrain),
        //         drivetrain.applyRequest(
        //                 () -> forwardStraight.withVelocityX(0).withVelocityY(-0.5)
        //         ).withTimeout(0.6)
        // ));

        // joystick.b().whileTrue(drivetrain.applyRequest(
        // () -> point.withModuleDirection(
        // new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))));

        joystick.pov(0).whileTrue(
                drivetrain.applyRequest(() -> forwardStraight.withVelocityX(0.5).withVelocityY(0)));
        joystick.pov(180)
                .whileTrue(drivetrain.applyRequest(
                        () -> forwardStraight.withVelocityX(-0.5).withVelocityY(0)));
        joystick.pov(90)
                .whileTrue(drivetrain.applyRequest(
                        () -> forwardStraight.withVelocityX(0).withVelocityY(-0.5)));

        joystick.pov(270)
                .whileTrue(drivetrain.applyRequest(
                        () -> forwardStraight.withVelocityX(0).withVelocityY(0.5)));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // reset the field-centric heading on left bumper press
        joystick.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        drivetrain.registerTelemetry(logger::telemeterize);

        elevatorUp.whileTrue(new ElevatorUpCommand(elevatorSubsystem, 0.5));
        elevatorDown.whileTrue(new ElevatorDownCommand(elevatorSubsystem, 0.25));

        intake.whileTrue(new IntakeOnCommand(intakeSubsystem));
        shoot.whileTrue(new IntakeShootCommand(intakeSubsystem));

        algeaPROCESSOR.onTrue(new AutoLifter(elevatorSubsystem, pivotSubsystem,
                ElevatorPositions.PROCESSOR, PivotPosition.PROCESSOR));
        algeaGROUND.onTrue(new AutoLifter(elevatorSubsystem, pivotSubsystem,
                ElevatorPositions.A1, PivotPosition.CLOSE));

        coralBASE.onTrue(
                new AutoLifter(elevatorSubsystem, pivotSubsystem,
                        ElevatorPositions.BASE,
                        PivotPosition.CLOSE));
        coralL1.onTrue(
                new AutoLifter(elevatorSubsystem, pivotSubsystem,
                        ElevatorPositions.L1,
                        PivotPosition.REEFL1));
        coralL2.onTrue(
                new AutoLifter(elevatorSubsystem, pivotSubsystem,
                        ElevatorPositions.L2,
                        PivotPosition.REEFL23));
        coralL3.onTrue(
                new AutoLifter(elevatorSubsystem, pivotSubsystem,
                        ElevatorPositions.L3,
                        PivotPosition.REEFL23));
        coralL4.onTrue(
        new AutoLifter(elevatorSubsystem, pivotSubsystem,
        ElevatorPositions.L4,
        PivotPosition.REEFL4));

        hpfeed.onTrue(new AutoLifter(elevatorSubsystem, pivotSubsystem,
                                ElevatorPositions.HP,
                                PivotPosition.HP));

    }

    public Command getAutonomousCommand() {
        /* Run the path selected from the auto chooser */
        return autoChooser.getSelected();
    }
}