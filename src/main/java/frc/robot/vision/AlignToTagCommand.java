package frc.robot.vision;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;

/**
 * Drives the robot to a fixed pose in front of whichever AprilTag the
 * Limelight currently sees.
 */
public class AlignToTagCommand extends Command {
    private static final String LIMELIGHT_NAME = "limelight";

    private static final AprilTagFieldLayout FIELD_LAYOUT =
        AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

    /**
     * Where the robot should end up, relative to the tag's pose: standing
     * this far in front of it, centered, facing it head-on. Tune per
     * mechanism (e.g. add a sideways offset for an arm that isn't centered).
     */
    private static final double STANDOFF_DISTANCE_METERS = 1.5;
    private static final Transform2d TAG_TO_TARGET = new Transform2d(
        new Translation2d(STANDOFF_DISTANCE_METERS, 0.0),
        Rotation2d.k180deg // robot faces back toward the tag
    );

    private static final double POSITION_TOLERANCE_METERS = 0.02;
    private static final double ROTATION_TOLERANCE_RADIANS = Math.toRadians(1.0);

    private final CommandSwerveDrivetrain drivetrain;

    private final PIDController xController = new PIDController(3.0, 0, 0);
    private final PIDController yController = new PIDController(3.0, 0, 0);
    private final PIDController thetaController = new PIDController(4.0, 0, 0);

    private final SwerveRequest.FieldCentric driveRequest = new SwerveRequest.FieldCentric();

    private Pose2d targetPose;

    public AlignToTagCommand(CommandSwerveDrivetrain drivetrain) {
        this.drivetrain = drivetrain;

        thetaController.enableContinuousInput(-Math.PI, Math.PI);
        xController.setTolerance(POSITION_TOLERANCE_METERS);
        yController.setTolerance(POSITION_TOLERANCE_METERS);
        thetaController.setTolerance(ROTATION_TOLERANCE_RADIANS);

        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        int tagId = (int) LimelightHelpers.getFiducialID(LIMELIGHT_NAME);
        targetPose = FIELD_LAYOUT.getTagPose(tagId)
            .map(tagPose -> tagPose.toPose2d().transformBy(TAG_TO_TARGET))
            .orElse(null); // no tag seen (id -1) or unknown id

        xController.reset();
        yController.reset();
        thetaController.reset();
    }

    @Override
    public void execute() {
        if (targetPose == null) {
            return;
        }

        Pose2d current = drivetrain.getState().Pose;

        double vx = xController.calculate(current.getX(), targetPose.getX());
        double vy = yController.calculate(current.getY(), targetPose.getY());
        double omega = thetaController.calculate(
            current.getRotation().getRadians(), targetPose.getRotation().getRadians());

        drivetrain.setControl(driveRequest
            .withVelocityX(vx)
            .withVelocityY(vy)
            .withRotationalRate(omega));
    }

    @Override
    public boolean isFinished() {
        if (targetPose == null) {
            return true; // nothing to align to, end immediately
        }
        return xController.atSetpoint() && yController.atSetpoint() && thetaController.atSetpoint();
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.setControl(new SwerveRequest.Idle());
    }
}
