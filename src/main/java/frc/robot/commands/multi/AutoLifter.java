// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.multi;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.elevator.ElevatorControlCommand;
import frc.robot.commands.pivot.AlgeaPivotControl;
import frc.robot.commands.pivot.CoralPivotControl;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem.ElevatorPositions;
import frc.robot.subsystems.pivot.AlgeaPivotSubsystem;
import frc.robot.subsystems.pivot.CoralPivotSubsystem;
import frc.robot.subsystems.pivot.AlgeaPivotSubsystem.AlgeaPivotPosition;
import frc.robot.subsystems.pivot.CoralPivotSubsystem.CoralPivotPosition;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoLifter extends SequentialCommandGroup {
  private ElevatorSubsystem elevatorSubsystem;
  private CoralPivotSubsystem coralPivotSubsystem;
  private AlgeaPivotSubsystem algeaPivotSubsystem;
  private ElevatorPositions elevatorLevel;
  private CoralPivotPosition coralPivotLevel;
  private AlgeaPivotPosition algeaPivotPosition;
  
  /** Creates a new L3. */
  public AutoLifter(ElevatorSubsystem elevatorSubsystem, CoralPivotSubsystem coralPivotSubsystem, AlgeaPivotSubsystem algeaPivotSubsystem, ElevatorPositions elevatorLevel,
  CoralPivotPosition coralPivotLevel, AlgeaPivotPosition algeaPivotLevel) {
    this.elevatorSubsystem = elevatorSubsystem;
    this.coralPivotSubsystem = coralPivotSubsystem;
    this.coralPivotLevel = coralPivotLevel;
    this.elevatorLevel = elevatorLevel;

    // Add your commands in the addCommands() call, e.g.
    // addComm                                            ands(new FooCommand(), new BarCommand());
    addCommands(
      new ParallelCommandGroup(
        new ElevatorControlCommand(elevatorSubsystem, elevatorLevel),
        new CoralPivotControl(coralPivotSubsystem, coralPivotLevel),
        new AlgeaPivotControl(algeaPivotSubsystem, algeaPivotLevel)
      )
    );  
  }
}
