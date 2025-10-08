package frc.robot.commands.pivot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.pivot.PivotSubsystem;
import frc.robot.subsystems.pivot.PivotSubsystem.PivotPosition;

public class PivotControl extends Command {
  private PivotSubsystem pivotSubsystem;
  private PivotPosition level;
  private double goalPos;


  public PivotControl(PivotSubsystem pivotSubsystem, PivotPosition level) {
    this.pivotSubsystem = pivotSubsystem;
    this.level = level;
    addRequirements(pivotSubsystem);
  }
  public void initialize() {
    switch (level) {

      case CLOSE:
        goalPos = Constants.PivotConstants.CLOSE;
        break;
      
      case NORMAL:
        goalPos = Constants.PivotConstants.NORMAL;
        break;

      case REEFL23:
        goalPos = Constants.PivotConstants.REEF_L2_L3;
        break;

      case REEFL4:
        goalPos = Constants.PivotConstants.REEF_L4;
        break;
      
      case PROCESSOR:
        goalPos = Constants.PivotConstants.PROCESSOR;
        break;
      
      case BARGE:
        goalPos = Constants.PivotConstants.BARGE;
        break;

      case HP:
        goalPos = Constants.PivotConstants.HP;
        break;

      case REEFL1:
        goalPos = Constants.PivotConstants.REEFL1;
        break;


      default:
        break;
    }
    pivotSubsystem.pivotController.setSetpoint(goalPos);
  }

  @Override
  public void execute() {
    double power = pivotSubsystem.pivotController.calculate(pivotSubsystem.getPivotPosition());

    power = power * 0.6;

    if(power < 0) {
      power = Math.max(-1.6, power);
      
    }else{
      power = Math.min(power, 1.6) * 1.3;
    }

    pivotSubsystem.pivotUp(power);
      
  }

  @Override
  public void end(boolean interrupted) {
    
    }

  @Override
  public boolean isFinished() {
    // Bu komut, buton basılı tutulduğu sürece devam eder.
    return false;
  }
}