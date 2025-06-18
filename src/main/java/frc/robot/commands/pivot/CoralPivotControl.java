package frc.robot.commands.pivot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.pivot.CoralPivotSubsystem;
import frc.robot.subsystems.pivot.CoralPivotSubsystem.CoralPivotPosition;

public class CoralPivotControl extends Command {
  private CoralPivotSubsystem coralPivotSubsystem;
  private CoralPivotPosition level;
  private double goalPos;


  public CoralPivotControl(CoralPivotSubsystem coralPivotSubsystem, CoralPivotPosition level) {
    this.coralPivotSubsystem = coralPivotSubsystem;
    this.level = level;
    addRequirements(coralPivotSubsystem);
  }
  public void initialize() {
    switch (level) {

      case CLOSE:
        goalPos = Constants.PivotConstants.CoralPivot.CLOSE_CORAL;
        break;
      
      case NORMAL:
        goalPos = Constants.PivotConstants.CoralPivot.NORMAL;
        break;

      case REEFL23:
        goalPos = Constants.PivotConstants.CoralPivot.REEF_L2_L3_CORAL;
        break;

      case REEFL4:
        goalPos = Constants.PivotConstants.CoralPivot.REEF_L4_CORAL;
        break;
      
      case HP:
        goalPos = Constants.PivotConstants.CoralPivot.HP_CORAL;
        break;

      case REEFL1:
        goalPos = Constants.PivotConstants.CoralPivot.REEFL1;
        break;


      default:
        break;
    }
    coralPivotSubsystem.pivotController.setSetpoint(goalPos);
  }

  @Override
  public void execute() {
    double power = coralPivotSubsystem.pivotController.calculate(coralPivotSubsystem.getPivotPosition());

    power = power * 0.6;

    if(power < 0) {
      power = Math.max(-1.6, power);
      
    }else{
      power = Math.min(power, 1.6) * 1.3;
    }

    coralPivotSubsystem.pivotUp(power);
      
    SmartDashboard.putNumber("CORAL", coralPivotSubsystem.getPivotPosition());
    SmartDashboard.putNumber("Adnanın gotunun powerı", power);
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