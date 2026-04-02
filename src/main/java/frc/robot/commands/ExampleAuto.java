// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.CANFuelSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ExampleAuto extends SequentialCommandGroup {
  /** Creates a new ExampleAuto. */
  public ExampleAuto(CommandSwerveDrivetrain driveSubsystem, CANFuelSubsystem ballSubsystem, ClimberSubsystem climb) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      driveSubsystem.runOnce(() -> driveSubsystem.seedFieldCentric(Rotation2d.kZero)),
      Commands.parallel(
        new ClimbUp(climb).withTimeout(4),
        Commands.sequence(
          new AutoDrive(driveSubsystem, 0.5,  -0.02).withTimeout(3.5),
          new AutoDrive(driveSubsystem, 0, 0).withTimeout(0.1)
        )
      ),
      new LaunchSequence(ballSubsystem, () -> (1)).withTimeout(6),
      new AutoDrive(driveSubsystem, 0.5, -0.02).withTimeout(1.8), // from 2.25
      new AutoDrive(driveSubsystem, 0, 0).withTimeout(0.1),
      new ClimbDown(climb).withTimeout(10)
    );

    
  }
}
