package frc.robot.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.CommandSwerveDrivetrain;

/**
 * Reads the AprilTag-based 3D robot pose from a Limelight and fuses it into
 * the drivetrain's pose estimator (passive correction only — this subsystem
 * never drives the robot).
 *
 * Uses the raw {@code botpose_wpiblue} solve (getBotPose3d_wpiBlue), not
 * MegaTag2, so it does not need SetRobotOrientation.
 */
public class VisionSubsystem extends SubsystemBase {
    private static final String LIMELIGHT_NAME = "limelight";

    private final CommandSwerveDrivetrain drivetrain;

    public VisionSubsystem(CommandSwerveDrivetrain drivetrain) {
        this.drivetrain = drivetrain;
    }

    @Override
    public void periodic() {
        // Guard against the "no tag seen" case: with no target, the underlying
        // NetworkTables array is all zeros, which decodes to Pose3d(0,0,0,no
        // rotation) — indistinguishable from actually being at the field
        // origin. getTV() is the only reliable way to know the pose is real.
        if (!LimelightHelpers.getTV(LIMELIGHT_NAME)) {
            return;
        }

        Pose2d visionPose = LimelightHelpers.getBotPose3d_wpiBlue(LIMELIGHT_NAME).toPose2d();

        // The pose was captured before "now" — back out capture + pipeline latency.
        double latencySeconds = (LimelightHelpers.getLatency_Capture(LIMELIGHT_NAME)
            + LimelightHelpers.getLatency_Pipeline(LIMELIGHT_NAME)) / 1000.0;
        double timestampSeconds = Timer.getFPGATimestamp() - latencySeconds;

        drivetrain.addVisionMeasurement(visionPose, timestampSeconds);
    }
}
