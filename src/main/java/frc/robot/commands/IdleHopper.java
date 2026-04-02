package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.HopperSubsystem;

public class IdleHopper extends Command {
    HopperSubsystem hopperSubsystem;

    public IdleHopper(HopperSubsystem hopperSubsystem) {
        addRequirements(hopperSubsystem);
        this.hopperSubsystem = hopperSubsystem;
    }

    public void initialize() {
        hopperSubsystem.stopMotor();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
