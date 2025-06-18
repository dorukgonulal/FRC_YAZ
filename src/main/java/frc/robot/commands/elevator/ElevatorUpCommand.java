package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.elevator.ElevatorSubsystem;

public class ElevatorUpCommand extends Command {
  private final ElevatorSubsystem elevator;
  private final double speed;

  /**
   * Asansörü yukarı hareket ettirmek için komut.
   * 
   * @param elevator ElevatorSubsystem referansı
   * @param speed Motor hızı (0 ile 1 arasında veya ihtiyaca göre)
   */
  public ElevatorUpCommand(ElevatorSubsystem elevator, double speed) {
    this.elevator = elevator;
    this.speed = speed;
    addRequirements(elevator); 
  }

  @Override
  public void execute() {
    // true parametresi asansörü kaldırmak için
    //if (elevator.getElevatorPosition()) {
      elevator.setMotors(speed);
    //} else {
      //elevator.setMotors(0);
    //}
  }

  @Override
  public void end(boolean interrupted) {
    // Komut sonlandığında asansörü durdur
    elevator.stallElevator(0.02);
  }

  @Override
  public boolean isFinished() {
    // Bu komut, buton basılı tutulduğu sürece devam eder.
    return false;
  }
}
