package frc.robot.commands.elevator;

import java.nio.channels.Pipe;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem.ElevatorPositions;

public class ElevatorControlCommand extends Command {

  private ElevatorSubsystem elevatorSubsystem;
  private ElevatorPositions level;
  private double goalPos;
  public double stallPower;
  public double slowPower = 1;

  public ElevatorControlCommand(ElevatorSubsystem elevatorSubsystem, ElevatorPositions level) {

    this.level = level;
    this.elevatorSubsystem = elevatorSubsystem;

    addRequirements(elevatorSubsystem);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    switch (level) {

      case BASE:
        goalPos = Constants.ElevatorConstants.ELEVATOR_SETPOINT_BASE - 0.07;
        break;
      
      case PROCESSOR:
        goalPos = Constants.ElevatorConstants.ELEVATOR_SETPOINT_PROCESSOR;
        break;

      case HP_FIRST:
        goalPos = Constants.ElevatorConstants.ELEVATOR_SETPOINT_HP_FIRST;
        stallPower = 0.02;
        break;
      
      case HP_LAST:
        goalPos = Constants.ElevatorConstants.ELEVATOR_SETPOINT_HP_LAST;
        stallPower = 0.02;
        break;

      case AUTOL3:
        goalPos = Constants.ElevatorConstants.ELEVATOR_SETPOINT_AUTOL3;
        break;

      case L1:
        goalPos = Constants.ElevatorConstants.ELEVATOR_SETPOINT_L1;
        stallPower = 0;
        break;

      case AUTOL2:
      goalPos = Constants.ElevatorConstants.ELEVATOR_AUTO_L2;
      stallPower = 0;
      break;

      case L2:
        goalPos = Constants.ElevatorConstants.ELEVATOR_SETPOINT_L2;
        stallPower = 0.;
        break;

      case L3:
        goalPos = Constants.ElevatorConstants.ELEVATOR_SETPOINT_L3;
        stallPower = 0;
        slowPower = 0.5;
        break;

      case L4:
        goalPos = Constants.ElevatorConstants.ELEVATOR_SETPOINT_L4;
        stallPower = 0.03;
        slowPower = 0.5;
        break;
      case MAX:
        goalPos = Constants.ElevatorConstants.ELEVATOR_SETPOINT_MAKS;
        break;
      default:
        break;
      case A1:
        goalPos = Constants.ElevatorConstants.ELEVATOR_SETPOINT_ALGEA1;
        break;
      case A2:
        goalPos = Constants.ElevatorConstants.ELEVATOR_SETPOINT_ALGEA2;
        break;
    }
    
    elevatorSubsystem.elevatorController.setSetpoint(goalPos);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double power = elevatorSubsystem.elevatorController.calculate(elevatorSubsystem.getElevatorPosition());
 
    //elevatorSubsystem.setMotors(power)
    
    if(power > 0){
      elevatorSubsystem.setMotors(power * slowPower);
    }

    if(power < 0){
      elevatorSubsystem.setMotors(power * 0.25
      
      );
    }
      

    SmartDashboard.putNumber("Elevator Encoder Position", elevatorSubsystem.getElevatorPosition());
    SmartDashboard.putNumber("Adnanın gotunun powerı", power);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    elevatorSubsystem.stallElevator(stallPower);
  }

  public boolean isFinished() {
    return elevatorSubsystem.atSetpoint();
  }
}
