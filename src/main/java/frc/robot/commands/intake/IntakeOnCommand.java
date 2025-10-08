package frc.robot.commands.intake;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import frc.robot.Constants;
import frc.robot.subsystems.intake.IntakeSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

public class IntakeOnCommand extends Command {
    private final IntakeSubsystem intakeSubsystem;

    public IntakeOnCommand(IntakeSubsystem intakeSubsystem) {
        this.intakeSubsystem = intakeSubsystem;
        addRequirements(intakeSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        //algeaSubsystem.setIdleMod(IdleMode.kBrake);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        intakeSubsystem.intakeOn(Constants.IntakeConstants.IntakePower);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        intakeSubsystem.stopIntake();
        // algeaSubsystem.setIdleMod(IdleMode.kCoast);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}