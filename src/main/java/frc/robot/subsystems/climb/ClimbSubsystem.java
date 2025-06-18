package frc.robot.subsystems.climb;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotID;

public class ClimbSubsystem extends SubsystemBase {

    private final SparkMax climbMotor = new SparkMax(33, MotorType.kBrushless);
    //private final SparkMaxConfig climbMotorConfig = new SparkMaxConfig();

    //private final CANSparkMax RollerTop = new CANSparkMax(RobotID.Intake.TOP_ROLLER_ID, MotorType.kBrushless);
    //private final CANSparkMax RollerBottom = new CANSparkMax(RobotID.Intake.BOTTOM_ROLLER_ID, MotorType.kBrushless);
    
    public ClimbSubsystem() {
        // climbMotorConfig
        //     .inverted(false)
        //     .idleMode(IdleMode.kBrake);
        // climbMotor.configure(climbMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // RollerTop.setInverted(false);
        // RollerBottom.setInverted(true);
        // RollerBottom.setIdleMode(IdleMode.kCoast);
        // RollerTop.setIdleMode(IdleMode.kCoast);
    }

    public void climb(double power) {
        climbMotor.set(-power);
    }

    public void unclimb(double power) {
        climbMotor.set(power);
    }

    
    public void hold(double power) {
        climbMotor.set(power);
    }

    public void stop() {
        climbMotor.stopMotor();
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
}