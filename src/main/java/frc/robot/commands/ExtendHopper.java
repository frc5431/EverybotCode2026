package frc.robot.commands;

import static frc.robot.Constants.HopperConstants.HOPPER_MAX_CURRENT;
import static frc.robot.Constants.HopperConstants.HOPPER_MOTOR_FORWARD_VOLTAGE;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.HopperSubsystem;

public class ExtendHopper extends Command {
    HopperSubsystem hopperSubsystem;

    public ExtendHopper(HopperSubsystem hopperSubsystem) {
        addRequirements(hopperSubsystem);
        this.hopperSubsystem = hopperSubsystem;
    }

    public void initialize() {
        hopperSubsystem.setHopperMotorVoltage(HOPPER_MOTOR_FORWARD_VOLTAGE);
    }

    @Override
    public void execute() {
        hopperSubsystem.setHopperMotorVoltage(HOPPER_MOTOR_FORWARD_VOLTAGE);
    }

    // @Override
    // public boolean isFinished() {
    //     return hopperSubsystem.getCurrent() > HOPPER_MAX_CURRENT;
    // }

    public void end(boolean interrupted) {
        hopperSubsystem.stopMotor();
    }
}
