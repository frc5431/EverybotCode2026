// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CANFuelSubsystem;
import static frc.robot.Constants.FuelConstants.*;

import java.util.function.DoubleSupplier;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class Launch extends Command {
  /** Creates a new Intake. */

  CANFuelSubsystem fuelSubsystem;
  DoubleSupplier triggerValue;

  public Launch(CANFuelSubsystem fuelSystem, DoubleSupplier triggerValue) {
    addRequirements(fuelSystem);
    this.fuelSubsystem = fuelSystem;
    this.triggerValue = triggerValue;
  }

  public AngularVelocity getMotorRPM() {
    final double val = this.triggerValue.getAsDouble();
    
    // Ayan implementation
    return Units.RPM.of(MIN_LAUNCHER_RPM + val * (MAX_LAUNCHER_RPM - MIN_LAUNCHER_RPM));
  }

  public void callMotor() {
    fuelSubsystem.setIntakeLauncherRollerVelocity(getMotorRPM());
    fuelSubsystem.setFeederRoller(SmartDashboard.getNumber("Launching feeder roller value", INDEXER_LAUNCHING_PERCENT));
  }

  // Called when the command is initially scheduled. Set the rollers to the
  // appropriate values for intaking
  @Override
  public void initialize() {callMotor();}

  // Called every time the scheduler runs while the command is scheduled. This
  // command doesn't require updating any values while running
  @Override
  public void execute() {callMotor();}

  // Called once the command ends or is interrupted. Stop the rollers
  @Override
  public void end(boolean interrupted) {
    fuelSubsystem.setIntakeLauncherRoller(0);
    fuelSubsystem.setFeederRoller(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}