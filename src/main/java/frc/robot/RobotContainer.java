// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import java.util.Random;
import java.util.Set;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.vision.AlignToTagCommand;
import frc.robot.vision.VisionSubsystem;

public class RobotContainer {
        private double MaxSpeed = 0.1 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired
                                                                                            // top
                                                                                            // speed
        private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per
                                                                                          // second
                                                                                          // max angular velocity

        /* Setting up bindings for necessary control of the swerve drive platform */
        private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
                .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
                .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive

        private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
        private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

        private final Telemetry logger = new Telemetry(MaxSpeed);

        private final CommandXboxController joystick = new CommandXboxController(0);

        public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
        private final VisionSubsystem vision = new VisionSubsystem(drivetrain);

        // Backup leg speed/ranges for the auto below (open-loop, not measured by
        // odometry -- distance is only approximate: seconds = meters / BACKUP_SPEED).
        private static final double BACKUP_SPEED_METERS_PER_SEC = 0.5;
        private static final double MIN_BACKUP_METERS = 0.5;
        private static final double MAX_BACKUP_METERS = 1.5;
        private static final double MAX_TURN_DEGREES = 45.0;

        private final Random random = new Random();

        public RobotContainer() {
                configureBindings();
        }

        private void configureBindings() {
                // Note that X is defined as forward according to WPILib convention,
                // and Y is defined as to the left according to WPILib convention.
                drivetrain.setDefaultCommand(
                                // Drivetrain will execute this command periodically
                                drivetrain.applyRequest(() -> drive.withVelocityX(-joystick.getLeftY() * MaxSpeed) // Drive
                                                .withVelocityY(-joystick.getLeftX() * MaxSpeed) // Drive left with
                                                                                                // negative X (left)
                                                .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive
                                ));

                // Idle while the robot is disabled. This ensures the configured
                // neutral mode is applied to the drive motors while disabled.
                final var idle = new SwerveRequest.Idle();
                RobotModeTriggers.disabled().whileTrue(
                                drivetrain.applyRequest(() -> idle).ignoringDisable(true));

                joystick.a().whileTrue(new AlignToTagCommand(drivetrain));
                joystick.b().whileTrue(drivetrain.applyRequest(
                                () -> point.withModuleDirection(
                                                new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))));

                // Run SysId routines when holding back/start and X/Y.
                // Note that each routine should be run exactly once in a single log.
                joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
                joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
                joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
                joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

                // Reset the field-centric heading on left bumper press.
                joystick.leftBumper().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

                drivetrain.registerTelemetry(logger::telemeterize);
        }

    /**
     * One backup leg: picks a random backup distance (MIN_BACKUP_METERS -
     * MAX_BACKUP_METERS) and a random turn (+/- MAX_TURN_DEGREES), then
     * drives it open-loop for however long that distance takes at
     * BACKUP_SPEED_METERS_PER_SEC. Built fresh each time it's called so the
     * numbers are re-rolled every time this leg runs.
     */
    private Command randomBackupLeg() {
        double distanceMeters = MIN_BACKUP_METERS
            + random.nextDouble() * (MAX_BACKUP_METERS - MIN_BACKUP_METERS);
        double turnDegrees = (random.nextDouble() * 2 - 1) * MAX_TURN_DEGREES; // [-MAX, MAX]

        double backupSeconds = distanceMeters / BACKUP_SPEED_METERS_PER_SEC;
        double rotationalRateRadPerSec = Math.toRadians(turnDegrees) / backupSeconds;

        return drivetrain.applyRequest(() -> drive
                .withVelocityX(-BACKUP_SPEED_METERS_PER_SEC)
                .withVelocityY(0)
                .withRotationalRate(rotationalRateRadPerSec))
                .withTimeout(backupSeconds);
    }

    /**
     * Test/demo auto: repeatedly backs up a random distance and turns a
     * random angle (open-loop, not closed-loop -- good enough for testing),
     * then re-aligns to whatever AprilTag is visible. Loops until the
     * autonomous period ends, at which point the scheduler cancels whatever
     * leg is currently running.
     */
    public Command getAutonomousCommand() {
        return Commands.repeatingSequence(
            Commands.defer(this::randomBackupLeg, Set.of(drivetrain)),
            drivetrain.applyRequest(() -> new SwerveRequest.Idle()),
            new AlignToTagCommand(drivetrain)
        );
    }
}