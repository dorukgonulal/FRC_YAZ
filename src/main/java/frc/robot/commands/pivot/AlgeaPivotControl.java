package frc.robot.commands.pivot;

import edu.wpi.first.units.measure.Power;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.pivot.AlgeaPivotSubsystem;
import frc.robot.subsystems.pivot.AlgeaPivotSubsystem.AlgeaPivotPosition;
import pabeles.concurrency.IntOperatorTask.Max;

public class AlgeaPivotControl extends Command {
  private AlgeaPivotSubsystem algeaPivotSubsystem;
  private AlgeaPivotPosition level;
  private double goalPos;


  public AlgeaPivotControl(AlgeaPivotSubsystem algeaPivotSubsystem, AlgeaPivotPosition level) {
    this.algeaPivotSubsystem = algeaPivotSubsystem;
    this.level = level;
    addRequirements(algeaPivotSubsystem);
  }
  public void initialize() {
    switch (level) {

      case CLOSE:
        goalPos = Constants.PivotConstants.AlgeaPivot.CLOSE;
        break;

      case REEFA1:
        goalPos = Constants.PivotConstants.AlgeaPivot.REEFA1;
        break;
      
        case REEFA2:
        goalPos = Constants.PivotConstants.AlgeaPivot.REEFA2;
        break;

      case PROCESSOR:
        goalPos = Constants.PivotConstants.AlgeaPivot.PROCESSOR;
        break;
      
      case GROUND:
        goalPos = Constants.PivotConstants.AlgeaPivot.GROUND;

      default:
        break;
    }
    algeaPivotSubsystem.pivotController.setSetpoint(goalPos);
  }

  @Override
  public void execute() {
    double power = algeaPivotSubsystem.pivotController.calculate(algeaPivotSubsystem.getPivotPosition());

    power = power * 0.6;

    if(power < 0) {
      power = Math.max(-1.6, power);
      
    }else{
      power = Math.min(power, 1.6) * 1.2;
    }

    algeaPivotSubsystem.pivotUp(power);
      
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