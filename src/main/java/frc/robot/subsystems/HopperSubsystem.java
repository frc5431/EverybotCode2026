package frc.robot.subsystems;

import static frc.robot.Constants.HopperConstants.*;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class HopperSubsystem extends SubsystemBase {
    private final SparkMax HopperMotor;
    private final SparkMax HopperBelt;

    public HopperSubsystem() {
        HopperMotor = new SparkMax(HOPPER_MOTOR_ID, MotorType.kBrushless);
        HopperBelt  = new SparkMax(HOPPER_BELT_ID,  MotorType.kBrushless);

        SparkMaxConfig hopperMotorConfig = new SparkMaxConfig();
        SparkMaxConfig hopperBeltConfig  = new SparkMaxConfig();

        hopperMotorConfig.smartCurrentLimit(HOPPER_MOTOR_CURRENT_LIMIT);
        hopperBeltConfig .smartCurrentLimit(HOPPER_BELT_CURRENT_LIMIT);

        HopperMotor.configure(hopperMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        HopperBelt .configure(hopperBeltConfig,  ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void extendHopperMotor(double speed) {
        // Turn exacly 90 deg (direction unknown)
        HopperMotor.set(+speed);
    }

    public void retractHopperMotor(double speed) {
        // Turn exacly 90 deg (direction unknown)
        HopperMotor.set(-speed);
    }
    // Direction unknown
    public void spinHopperBelt(double speed) {HopperMotor.set(speed);}
    public void stopHopperBelt() {HopperBelt.set(0);}
}
