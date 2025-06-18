package frc.robot.subsystems.intake;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotID;

public class CoralSubsystem extends SubsystemBase {

    private final SparkMax RollerLeft = new SparkMax(RobotID.Intake.Coral.ROLLER_LEFT, MotorType.kBrushless);
    private final SparkMax RollerRight = new SparkMax(RobotID.Intake.Coral.ROLLER_RIGHT, MotorType.kBrushless);
    // private final SparkMaxConfig configRollerLeft = new SparkMaxConfig();
    // private final SparkMaxConfig configRollerRight = new SparkMaxConfig();

    //private final CANSparkMax RollerTop = new CANSparkMax(RobotID.Intake.TOP_ROLLER_ID, MotorType.kBrushless);
    //private final CANSparkMax RollerBottom = new CANSparkMax(RobotID.Intake.BOTTOM_ROLLER_ID, MotorType.kBrushless);

    public CoralSubsystem() {
        // configRollerLeft
        //     .inverted(false)
        //     .idleMode(IdleMode.kCoast);
        // RollerLeft.configure(configRollerLeft, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters); 

        // configRollerRight
        //     .inverted(false)
        //     .idleMode(IdleMode.kCoast);
        // RollerRight.configure(configRollerRight, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // RollerTop.setInverted(false);
        // RollerBottom.setInverted(true);
        // RollerBottom.setIdleMode(IdleMode.kCoast);
        // RollerTop.setIdleMode(IdleMode.kCoast);
    }

    public void intakeOn(double power) {
        RollerLeft.set(power);
        RollerRight.set(-power);
    }

    public void intakeShoot(double power) {
        RollerLeft.set(-power);
        RollerRight.set(power);
    }

    // public void setIdleMod(IdleMode idleMode) {
    //     configRollerLeft
    //         .idleMode(IdleMode.kCoast);
    //     RollerLeft.configure(configRollerLeft, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    //     configRollerRight
    //         .idleMode(IdleMode.kCoast);
    //     RollerRight.configure(configRollerRight, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    // }

    public void stopIntake() {
        RollerLeft.stopMotor();
        RollerRight.stopMotor();
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
}