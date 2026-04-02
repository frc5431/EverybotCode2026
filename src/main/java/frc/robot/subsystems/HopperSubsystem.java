package frc.robot.subsystems;

import static frc.robot.Constants.HopperConstants.*;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class HopperSubsystem extends SubsystemBase {
    private final SparkMax hopperMotor;

    public HopperSubsystem() {
        hopperMotor = new SparkMax(HOPPER_MOTOR_ID, MotorType.kBrushless);
        SparkMaxConfig hopperMotorConfig = new SparkMaxConfig();
        hopperMotorConfig.smartCurrentLimit(HOPPER_MOTOR_CURRENT_LIMIT);
        hopperMotor.configure(hopperMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void setHopperMotor(double power) {
        hopperMotor.set(power);
    }

    public void setHopperMotorVoltage(double power) {
        hopperMotor.setVoltage(power);
    }

    public void stopMotor() {
        hopperMotor.set(0);
    }

    public double getCurrent() {
        return hopperMotor.getOutputCurrent();
    }

    @Override
    public void periodic() {
        Logger.recordOutput("Hopper/Voltage", hopperMotor.getBusVoltage() * hopperMotor.getAppliedOutput());
        Logger.recordOutput("Hopper/Velocity", hopperMotor.getEncoder().getVelocity());
        Logger.recordOutput("Hopper/SupplyCurrent", hopperMotor.getOutputCurrent() * hopperMotor.getAppliedOutput());
        Logger.recordOutput("Hopper/StatorCurrent", hopperMotor.getOutputCurrent());
    }
}
