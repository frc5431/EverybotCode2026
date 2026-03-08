// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CANFuelSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.commands.ClimbDown;
import frc.robot.commands.ClimbUp;
import frc.robot.commands.Eject;
import frc.robot.commands.ExampleAuto;
import frc.robot.commands.Intake;
import frc.robot.commands.Launch;
import frc.robot.commands.LaunchSequence;

public class RobotContainer {
    private double FastMaxSpeed = 0.75 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double FastMaxAngularRate = 0.3*RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    private double SlowMaxSpeed = 0.15 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double SlowMaxAngularRate = 0.3*RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity
    
    
    private double MaxSpeed = 0.5 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = 0.3*RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController driveJoystick = new CommandXboxController(0);
    private final CommandXboxController operatorJoystick = new CommandXboxController(1);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    public final CANFuelSubsystem canFuelSubsystem = new CANFuelSubsystem();
    public final ClimberSubsystem climbSubsystem = new ClimberSubsystem();

    public RobotContainer() {
        configureBindings();
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-MathUtil.applyDeadband(-driveJoystick.getLeftY(), 0.1, 1) * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-MathUtil.applyDeadband(-driveJoystick.getLeftX(), 0.1, 1) * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-MathUtil.applyDeadband(driveJoystick.getRightX(), 0.1, 1) * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        driveJoystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
        driveJoystick.b().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-driveJoystick.getLeftY(), -driveJoystick.getLeftX()))
        ));

        operatorJoystick.a();

        // Reset the field-centric heading on left bumper press.
        driveJoystick.leftBumper().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

        drivetrain.registerTelemetry(logger::telemeterize);

// NOT TESTED STARTING HERE AND BELOW





//Apply rotation left when left dpad is pressed
        driveJoystick.povLeft()
            .onTrue(new RunCommand(() -> {
              
                
            drivetrain.applyRequest(() ->
                drive.withRotationalRate(-0.1) // Drive counterclockwise with negative X (left)
            );


            }, drivetrain).withTimeout(0.1));

//Apply rotation right when right dpad is pressed         
driveJoystick.povRight()
            .onTrue(new RunCommand(() -> {
              
                
            drivetrain.applyRequest(() ->
                drive.withRotationalRate(0.1) // Drive counterclockwise with negative X (left)
            );


            }, drivetrain).withTimeout(0.1));




//while right trigger held make driving faster

        driveJoystick.rightTrigger(0.8)
            .whileTrue(new RunCommand(() -> {
              
                
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-MathUtil.applyDeadband(-driveJoystick.getLeftY(), 0.1, 1) * FastMaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-MathUtil.applyDeadband(-driveJoystick.getLeftX(), 0.1, 1) * FastMaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-MathUtil.applyDeadband(driveJoystick.getRightX(), 0.1, 1) * FastMaxAngularRate) // Drive counterclockwise with negative X (left)
            );


            }));


//while right trigger held make driving slower
            driveJoystick.leftTrigger(0.8)
            .whileTrue(new RunCommand(() -> {
              
                
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-MathUtil.applyDeadband(-driveJoystick.getLeftY(), 0.1, 1) * SlowMaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-MathUtil.applyDeadband(-driveJoystick.getLeftX(), 0.1, 1) * SlowMaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-MathUtil.applyDeadband(driveJoystick.getRightX(), 0.1, 1) * SlowMaxAngularRate) // Drive counterclockwise with negative X (left)
            );


            }, drivetrain));
    
    // END OF NOT TESTED        


        

        operatorJoystick.axisLessThan(XboxController.Axis.kRightY.value, -0.8)
            .whileTrue(new Eject(canFuelSubsystem));
        
        operatorJoystick.axisGreaterThan(XboxController.Axis.kRightY.value, 0.8)
            .whileTrue(new Intake(canFuelSubsystem));

        operatorJoystick.axisGreaterThan(XboxController.Axis.kLeftY.value, 0.8)
            .whileTrue(new ClimbUp(climbSubsystem));
        
        operatorJoystick.axisLessThan(XboxController.Axis.kLeftY.value, -0.8)
            .whileTrue(new ClimbDown(climbSubsystem));
        
        operatorJoystick.rightTrigger(0.8)
            .whileTrue(new LaunchSequence(canFuelSubsystem));

        operatorJoystick.leftTrigger(0.8)
            .whileTrue(new RunCommand(() -> {
                canFuelSubsystem.setFeederRoller(0);
                canFuelSubsystem.setIntakeLauncherRoller(0);
            }, canFuelSubsystem)).onFalse(Commands.runOnce(
                () -> {
                    canFuelSubsystem.setFeederRoller(0);
                    canFuelSubsystem.setIntakeLauncherRoller(0);
                }, canFuelSubsystem));
    }

    public Command getAutonomousCommand() {
        // Simple drive forward auton
        // final var idle = new SwerveRequest.Idle();
        // return Commands.sequence(
        //     // Reset our field centric heading to match the robot
        //     // facing away from our alliance station wall (0 deg).
        //     drivetrain.runOnce(() -> drivetrain.seedFieldCentric(Rotation2d.kZero)),
        //     // Then slowly drive forward (away from us) for 5 seconds.
        //     drivetrain.applyRequest(() ->
        //         drive.withVelocityX(0.5)
        //             .withVelocityY(0)
        //             .withRotationalRate(0)
        //     )
        //     .withTimeout(5.0),
        //     // Finally idle for the rest of auton
        //     drivetrain.applyRequest(() -> idle)
        // );
        // return new ExampleAuto(drivetrain, canFuelSubsystem, climbSubsystem);
        return Commands.none();
    }
}
