package frc.robot.commands.intake;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import frc.robot.Constants;
import frc.robot.subsystems.intake.AlgeaSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

public class AlgeaIntakeOnCommand extends Command {
    private final AlgeaSubsystem algeaSubsystem;

    public AlgeaIntakeOnCommand(AlgeaSubsystem algeaSubsystem) {
        this.algeaSubsystem = algeaSubsystem;
        addRequirements(algeaSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        //algeaSubsystem.setIdleMod(IdleMode.kBrake);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        algeaSubsystem.intakeOn(Constants.IntakeConstants.CoralConstants.IntakePower);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        algeaSubsystem.stopIntake();
        // algeaSubsystem.setIdleMod(IdleMode.kCoast);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}