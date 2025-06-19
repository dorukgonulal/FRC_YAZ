package frc.robot.commands.intake;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import frc.robot.Constants;
import frc.robot.subsystems.intake.CoralSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

public class CoralSlowShootCommand extends Command {
    private final CoralSubsystem coralSubsystem;

    public CoralSlowShootCommand(CoralSubsystem coralSubsystem) {
        this.coralSubsystem = coralSubsystem;
        addRequirements(coralSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        //coralSubsystem.setIdleMod(IdleMode.kBrake);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        coralSubsystem.intakeShoot(0.15);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        coralSubsystem.stopIntake();
        // coralSubsystem.setIdleMod(IdleMode.kCoast);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}