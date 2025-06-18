package frc.robot.subsystems.intake;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotID;

public class AlgeaSubsystem extends SubsystemBase {

    private final SparkMax RollerLeft = new SparkMax(RobotID.Intake.Algea.ROLLER_LEFT, MotorType.kBrushless);
    private final SparkMax RollerRight = new SparkMax(RobotID.Intake.Algea.ROLLER_RIGHT, MotorType.kBrushless);
    private final RelativeEncoder rollerleftencoder = RollerLeft.getEncoder();
    private final RelativeEncoder rollerrightencoder = RollerRight.getEncoder();

    public AlgeaSubsystem() {}

    public void intakeOn(double power) {
        RollerLeft.set(power);
        RollerRight.set(-power);
    }

    public void intakeShoot(double power) {
        RollerLeft.set(-power);
        RollerRight.set(power);
    }

    public void stopIntake() {
        RollerLeft.stopMotor();
        RollerRight.stopMotor();
    }

    public double getleftposition() {
        return rollerleftencoder.getPosition();
    }

    public double getrightposition() {
        return rollerrightencoder.getPosition();
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("ROLLERLEFT", getleftposition());
        SmartDashboard.putNumber("ROLLERRIGHT", getrightposition());

    }
}