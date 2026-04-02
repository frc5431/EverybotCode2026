// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

// import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
// import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularVelocity;
// import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.LoggedTunableNumber;

import static frc.robot.Constants.FuelConstants.*;

import org.littletonrobotics.junction.Logger;

public class CANFuelSubsystem extends SubsystemBase {
  private final SparkMax LeftLeaderIntakeLauncher;
  private final SparkMax RightFollowerIntakeLauncher;
  private final SparkMax Indexer;
  private final SimpleMotorFeedforward ff = new SimpleMotorFeedforward(ks_SHOOTER_PID.get(), kv_SHOOTER_PID.get(), ka_SHOOTER_PID.get());
  private final PIDController pid = new PIDController(kp_SHOOTER_PID.get(), ki_SHOOTER_PID.get(), kd_SHOOTER_PID.get());

  /** Creates a new CANBallSubsystem. */
  public CANFuelSubsystem() {
    // create brushed motors for each of the motors on the launcher mechanism
    LeftLeaderIntakeLauncher = new SparkMax(LEFT_INTAKE_LAUNCHER_MOTOR_ID, MotorType.kBrushless);
    RightFollowerIntakeLauncher = new SparkMax(RIGHT_INTAKE_LAUNCHER_MOTOR_ID, MotorType.kBrushless);
    Indexer = new SparkMax(INDEXER_MOTOR_ID, MotorType.kBrushless);

    // create the configuration for the feeder roller, set a current limit and apply
    // the config to the controller
    SparkMaxConfig feederConfig = new SparkMaxConfig();
    feederConfig.smartCurrentLimit(INDEXER_MOTOR_CURRENT_LIMIT);
    Indexer.configure(feederConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // create the configuration for the launcher roller, set a current limit, set
    // the motor to inverted so that positive values are used for both intaking and
    // launching, and apply the config to the controller
    SparkMaxConfig leaderLauncherConfig = new SparkMaxConfig();
    leaderLauncherConfig.smartCurrentLimit(LAUNCHER_MOTOR_CURRENT_LIMIT);
    leaderLauncherConfig.voltageCompensation(12);
    leaderLauncherConfig.idleMode(IdleMode.kCoast);
    leaderLauncherConfig.inverted(true);
    
    SparkMaxConfig followerLauncherConfig = new SparkMaxConfig();
    followerLauncherConfig.smartCurrentLimit(LAUNCHER_MOTOR_CURRENT_LIMIT);
    followerLauncherConfig.voltageCompensation(12);
    followerLauncherConfig.idleMode(IdleMode.kCoast);
    followerLauncherConfig.follow(LeftLeaderIntakeLauncher, true);

    LeftLeaderIntakeLauncher.configure(leaderLauncherConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    RightFollowerIntakeLauncher.configure(followerLauncherConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // put default values for various fuel operations onto the dashboard
    // all commands using this subsystem pull values from the dashbaord to allow
    // you to tune the values easily, and then replace the values in Constants.java
    // with your new values. For more information, see the Software Guide.
    SmartDashboard.putNumber("Intaking feeder roller value", INDEXER_INTAKING_PERCENT);
    SmartDashboard.putNumber("Intaking intake roller value", INTAKE_INTAKING_PERCENT);
    SmartDashboard.putNumber("Launching feeder roller value", INDEXER_LAUNCHING_PERCENT);
    SmartDashboard.putNumber("Launching launcher roller value", LAUNCHING_LAUNCHER_PERCENT);
    //SmartDashboard.putNumber("Spin-up feeder roller value", SPIN_UP_FEEDER_VOLTAGE);
  }

  // A method to set the voltage of the intake roller
  public void setIntakeLauncherRoller(double power) {
    LeftLeaderIntakeLauncher.set(power);
    // RightFollowerIntakeLauncher.set(power); // positive for shooting
  }

  public void setIntakeLauncherRollerVelocity(AngularVelocity desired) {
    Logger.recordOutput("Shooter/DesiredSpeed", desired.in(Units.RPM));
    LoggedTunableNumber.ifChanged(hashCode(), (args) -> {
        ff.setKs(args[0]);
        ff.setKv(args[1]);
        ff.setKa(args[2]);
    }, ks_SHOOTER_PID, kv_SHOOTER_PID, ka_SHOOTER_PID);

    LoggedTunableNumber.ifChanged(hashCode(), (args) -> {
        pid.setP(args[0]);
        pid.setI(args[1]);
        pid.setD(args[2]);
    }, kp_SHOOTER_PID, ki_SHOOTER_PID, kd_SHOOTER_PID);

    double ffVoltage = ff.calculate(desired.in(Units.RPM));
    Logger.recordOutput("Shooter/ffVoltage", ffVoltage);
    double current = LeftLeaderIntakeLauncher.getEncoder().getVelocity();
    Logger.recordOutput("Shooter/CurrentSpeed", current);
    double pidVoltage = pid.calculate(current, desired.in(Units.RPM));
    Logger.recordOutput("Shooter/pidVoltage", pidVoltage);

    LeftLeaderIntakeLauncher.setVoltage(ffVoltage + pidVoltage);
  }

  // A method to set the voltage of the intake roller
  public void setFeederRoller(double power) {
    Indexer.set(power); // positive for shooting
  }

  // A method to stop the rollers
  public void stop() {
    Indexer.set(0);
    LeftLeaderIntakeLauncher.set(0);
    // RightFollowerIntakeLauncher.set(0);
  }

  @Override
  public void periodic() {
    // All motors: applied voltage, velocity, supply current, stator current
    Logger.recordOutput("Launcher/Left/Voltage", LeftLeaderIntakeLauncher.getBusVoltage() * LeftLeaderIntakeLauncher.getAppliedOutput());
    Logger.recordOutput("Launcher/Left/Velocity", LeftLeaderIntakeLauncher.getEncoder().getVelocity());
    Logger.recordOutput("Launcher/Left/SupplyCurrent", LeftLeaderIntakeLauncher.getOutputCurrent() * LeftLeaderIntakeLauncher.getAppliedOutput());
    Logger.recordOutput("Launcher/Left/StatorCurrent", LeftLeaderIntakeLauncher.getOutputCurrent());
    
    Logger.recordOutput("Launcher/Right/Voltage", RightFollowerIntakeLauncher.getBusVoltage() * RightFollowerIntakeLauncher.getAppliedOutput());
    Logger.recordOutput("Launcher/Right/Velocity", RightFollowerIntakeLauncher.getEncoder().getVelocity());
    Logger.recordOutput("Launcher/Right/SupplyCurrent", RightFollowerIntakeLauncher.getOutputCurrent() * RightFollowerIntakeLauncher.getAppliedOutput());
    Logger.recordOutput("Launcher/Right/StatorCurrent", RightFollowerIntakeLauncher.getOutputCurrent());
    
    Logger.recordOutput("Launcher/Indexer/Voltage", Indexer.getBusVoltage() * Indexer.getAppliedOutput());
    Logger.recordOutput("Launcher/Indexer/Velocity", Indexer.getEncoder().getVelocity());
    Logger.recordOutput("Launcher/Indexer/SupplyCurrent", Indexer.getOutputCurrent() * Indexer.getAppliedOutput());
    Logger.recordOutput("Launcher/Indexer/StatorCurrent", Indexer.getOutputCurrent());
  }
}