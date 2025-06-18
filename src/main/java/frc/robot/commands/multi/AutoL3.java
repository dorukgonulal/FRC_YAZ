// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.multi;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.elevator.ElevatorControlCommand;
import frc.robot.commands.intake.CoralIntakeOnCommand;
import frc.robot.commands.intake.CoralShootCommand;
import frc.robot.commands.pivot.AlgeaPivotControl;
import frc.robot.commands.pivot.CoralPivotControl;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem.ElevatorPositions;
import frc.robot.subsystems.intake.CoralSubsystem;
import frc.robot.subsystems.pivot.AlgeaPivotSubsystem;
import frc.robot.subsystems.pivot.CoralPivotSubsystem;
import frc.robot.subsystems.pivot.AlgeaPivotSubsystem.AlgeaPivotPosition;
import frc.robot.subsystems.pivot.CoralPivotSubsystem.CoralPivotPosition;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoL3 extends Command {
    private ElevatorSubsystem elevatorSubsystem;
    private CoralPivotSubsystem coralPivotSubsystem;
    private CoralSubsystem coralSubsystem;
    DigitalInput coralSwitch = new DigitalInput(0);

    /** Creates a new L3. */
    public AutoL3(ElevatorSubsystem elevatorSubsystem, CoralPivotSubsystem coralPivotSubsystem) {
        this.elevatorSubsystem = elevatorSubsystem;
        this.coralPivotSubsystem = coralPivotSubsystem;
    }

    @Override
    public void execute() {
        if(!coralSwitch.get()) {
            new ParallelCommandGroup(
                new ElevatorControlCommand(elevatorSubsystem, ElevatorPositions.BASE),
                new CoralPivotControl(coralPivotSubsystem, CoralPivotPosition.CLOSE)
            );
        } else {
            new ParallelCommandGroup(
                new ElevatorControlCommand(elevatorSubsystem, ElevatorPositions.L3),
                new CoralPivotControl(coralPivotSubsystem, CoralPivotPosition.REEFL23),
                
                new SequentialCommandGroup(
                    new CoralShootCommand(coralSubsystem),
                    new WaitCommand(0.5)
                )
            );
        }
    }
}
