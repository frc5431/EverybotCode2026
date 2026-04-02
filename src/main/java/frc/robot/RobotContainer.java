// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import javax.print.attribute.standard.MediaSize.NA;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CANFuelSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIOLimelight;
import frc.robot.utils.FieldConstants;
import frc.robot.commands.ClimbDown;
import frc.robot.commands.ClimbUp;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.Eject;
import frc.robot.commands.ExampleAuto;
import frc.robot.commands.ExtendHopper;
import frc.robot.commands.IdleClimb;
import frc.robot.commands.IdleHopper;
import frc.robot.commands.IdleLaunch;
import frc.robot.commands.Intake;
import frc.robot.commands.Launch;
import frc.robot.commands.LaunchSequence;
import frc.robot.commands.RetractHopper;
import frc.robot.commands.Tuning;


import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import frc.robot.utils.AllianceFlipUtil;

public class RobotContainer {
    // private double FastMaxSpeed = 0.9 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    // private double FastMaxAngularRate = 0.7 * RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    // private double SlowMaxSpeed = 0.15 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    // private double SlowMaxAngularRate = 0.2 * RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity
    
    
    private double MaxSpeed = 0.75
     * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = 1*RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);
    private LoggedDashboardChooser<Command> autoChooser;

    private final CommandXboxController driveJoystick = new CommandXboxController(0);
    private final CommandXboxController operatorJoystick = new CommandXboxController(1);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    public final CANFuelSubsystem canFuelSubsystem = new CANFuelSubsystem();
    public final ClimberSubsystem climbSubsystem = new ClimberSubsystem();
    public final HopperSubsystem hopperSubsystem = new HopperSubsystem();
    public final Vision vision = new Vision((visionRobotPoseMeters, timestampSeconds, visionMeasurementStdDevs) -> {
        drivetrain.addVisionMeasurement(visionRobotPoseMeters, timestampSeconds, visionMeasurementStdDevs);
    }, new VisionIOLimelight("Limelight", () -> (drivetrain.getState().Pose.getRotation())));
    


    public RobotContainer() {
        configureBindings();
        configureDefaultCommands();
        configureNamedCommands();
    }

    public void configureDefaultCommands() {
        canFuelSubsystem.setDefaultCommand(new IdleLaunch(canFuelSubsystem));
        climbSubsystem.setDefaultCommand(new IdleClimb(climbSubsystem));
        hopperSubsystem.setDefaultCommand(new IdleHopper(hopperSubsystem));
    }

    public double getDriveSpeed() {
        SmartDashboard.putNumber("Left trigger axis", driveJoystick.getLeftTriggerAxis());
        SmartDashboard.putNumber("Left trigger thing", 0.5 - (driveJoystick.getLeftTriggerAxis() / 4));
        SmartDashboard.putNumber("Right trigger axis", driveJoystick.getRightTriggerAxis());
        SmartDashboard.putNumber("Right trigger thing", 0.5 - (driveJoystick.getRightTriggerAxis() / 4));

        if (driveJoystick.getRightTriggerAxis() > 0.1) {
            return (0.5 + (driveJoystick.getRightTriggerAxis() / 2)) 
            * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
        }
        
        if (driveJoystick.getLeftTriggerAxis() > 0.1) {
            return (0.5 - (driveJoystick.getLeftTriggerAxis() / 4)) 
            * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
        }

        return MaxSpeed;
    }

    private void configureBindings() {

        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        DoubleSupplier driveSpeed = () -> (getDriveSpeed());
        DoubleSupplier driveAngle = () -> (MaxAngularRate);
        
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(MathUtil.applyDeadband(-driveJoystick.getLeftY(), 0.1, 1) * driveSpeed.getAsDouble()) // Drive forward with negative Y (forward)
                    .withVelocityY(MathUtil.applyDeadband(-driveJoystick.getLeftX(), 0.1, 1) * driveSpeed.getAsDouble()) // Drive left with negative X (left)
                    .withRotationalRate(-MathUtil.applyDeadband(driveJoystick.getRightX(), 0.1, 1) * driveAngle.getAsDouble()) // Drive counterclockwise with negative X (left)
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

        // Reset the field-centric heading on left bumper press.
        driveJoystick.start().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

        driveJoystick.x().whileTrue(DriveCommands.joystickDriveAtAngle(
            drivetrain, () -> -driveJoystick.getLeftY(), () -> -driveJoystick.getLeftX(), 
            () -> getTranslationToHub().getAngle()));

        drivetrain.registerTelemetry(logger::telemeterize);

        operatorJoystick.axisLessThan(XboxController.Axis.kRightY.value, -0.8)
            .whileTrue(new Eject(canFuelSubsystem));
        
        operatorJoystick.axisGreaterThan(XboxController.Axis.kRightY.value, 0.8)
            .whileTrue(new Intake(canFuelSubsystem));

        operatorJoystick.axisGreaterThan(XboxController.Axis.kLeftY.value, 0.8)
            .whileTrue(new ClimbUp(climbSubsystem));
        
        operatorJoystick.axisLessThan(XboxController.Axis.kLeftY.value, -0.8)
            .whileTrue(new ClimbDown(climbSubsystem));
        
        operatorJoystick.a().whileTrue(new Tuning(canFuelSubsystem));
        operatorJoystick.rightBumper().whileTrue(new ExtendHopper(hopperSubsystem));
        operatorJoystick.leftBumper().whileTrue(new RetractHopper(hopperSubsystem));
        
        // operatorJoystick.rightTrigger(0.8)
        //     .whileTrue(new LaunchSequence(canFuelSubsystem));

        operatorJoystick.rightTrigger(0.1)
            .whileTrue(new LaunchSequence(canFuelSubsystem, () -> {return operatorJoystick.getRightTriggerAxis();}));

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

    public void configureNamedCommands() {
        // Fixed commands
        NamedCommands.registerCommand("LaunchSequence", new LaunchSequence(canFuelSubsystem, () -> (1)));
        NamedCommands.registerCommand("ClimbUp", new ClimbUp(climbSubsystem));
        NamedCommands.registerCommand("ClimbDown", new ClimbDown(climbSubsystem));
        NamedCommands.registerCommand("Intake", new Intake(canFuelSubsystem));
        NamedCommands.registerCommand("Eject", new Eject(canFuelSubsystem));
        NamedCommands.registerCommand("ExtendHopper", new ExtendHopper(hopperSubsystem));
        NamedCommands.registerCommand("RetractHopper", new RetractHopper(hopperSubsystem));
        NamedCommands.registerCommand("SeedBackward", drivetrain.runOnce(
            () -> drivetrain.seedFieldCentric(Rotation2d.k180deg)));
        NamedCommands.registerCommand("AutoAlignHub", DriveCommands.joystickDriveAtAngle(
            drivetrain, () -> 0, () -> 0, () -> getTranslationToHub().getAngle()));
        
        // Auton chooser
        autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());
        autoChooser.addOption("nothing", Commands.none());
        SmartDashboard.putData("AutonScheduler", CommandScheduler.getInstance());
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
        // return new PathPlannerAuto("TestAuto");
        return autoChooser.get();
    }

    public Translation2d getTranslationToHub() {
        Translation2d hubPose = FieldConstants.Hub.innerCenterPoint.toTranslation2d();
        Translation2d hubPoseAdj = AllianceFlipUtil.apply(hubPose);
        
        Pose2d shooterPose = drivetrain.getState().Pose;
        Translation2d diff = new Translation2d(0, 0).minus(hubPoseAdj.minus(shooterPose.getTranslation()));
        Logger.recordOutput("/Measurements/DistToHub", diff.getNorm());
        Logger.recordOutput("/Measurements/AngleToHub", diff.getAngle());
        return diff;
    }
}
