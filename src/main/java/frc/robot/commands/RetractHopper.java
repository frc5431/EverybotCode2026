package frc.robot.commands;

import static frc.robot.Constants.HopperConstants.HOPPER_MAX_CURRENT;
import static frc.robot.Constants.HopperConstants.HOPPER_MOTOR_BACKWARD_VOLTAGE;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.HopperSubsystem;

public class RetractHopper extends Command {
    HopperSubsystem hopperSubsystem;

    public RetractHopper(HopperSubsystem hopperSubsystem) {
        addRequirements(hopperSubsystem);
        this.hopperSubsystem = hopperSubsystem;
    }

    public void initialize() {
        hopperSubsystem.setHopperMotorVoltage(-1 * HOPPER_MOTOR_BACKWARD_VOLTAGE);
    }

    public void execute() {
        hopperSubsystem.setHopperMotorVoltage(-1 * HOPPER_MOTOR_BACKWARD_VOLTAGE);
    }

    @Override
    public boolean isFinished() {
        return hopperSubsystem.getCurrent() > HOPPER_MAX_CURRENT;
    }

    public void end(boolean interrupted) {
        hopperSubsystem.stopMotor();
    }
}
