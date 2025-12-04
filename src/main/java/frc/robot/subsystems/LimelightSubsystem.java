package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.PoseEstimate;
import frc.robot.LimelightHelpers.RawFiducial;
import java.util.Optional;

/**
 * Limelight vision subsystem modeled after the pose-consumer patterns used by the reference teams
 * in {@code useful-repo/C2024-Public}. It polls pose estimates, filters out low-confidence
 * measurements, and exposes the latest trusted result for downstream odometry/path-planning code.
 */
public class LimelightSubsystem extends SubsystemBase {
  /** Simple container for the last usable vision sample. */
  public static record VisionObservation(
      Pose2d poseMeters,
      double timestampSeconds,
      double xyStdDevMeters,
      double thetaStdDevRad,
      int tagCount,
      double avgTagDistance,
      double avgTagArea,
      double avgAmbiguity,
      boolean isMegaTag2) {}

  private VisionObservation latestObservation;
  private double lastHeartbeat = 0.0;

  @Override
  public void periodic() {
    PoseEstimate estimate = sampleEstimate();
    if (!isEstimateUsable(estimate)) {
      return;
    }

    double avgAmbiguity = computeAverageAmbiguity(estimate);
    if (avgAmbiguity > Constants.LimelightConstants.MAX_POSE_AMBIGUITY) {
      return;
    }

    MeasurementNoise noise = estimateNoise(estimate);
    latestObservation =
        new VisionObservation(
            estimate.pose,
            estimate.timestampSeconds,
            noise.xyStdDev,
            noise.thetaStdDev,
            estimate.tagCount,
            estimate.avgTagDist,
            estimate.avgTagArea,
            avgAmbiguity,
            estimate.isMegaTag2);
    lastHeartbeat = Timer.getFPGATimestamp();
  }

  /** Returns an optional view of the cached observation. */
  public Optional<VisionObservation> getLatestObservation() {
    return Optional.ofNullable(latestObservation);
  }

  /** Returns the cached pose if present. */
  public Optional<Pose2d> getLatestPose() {
    return getLatestObservation().map(VisionObservation::poseMeters);
  }

  /** Whether a measurement has been produced recently. */
  public boolean hasFreshObservation(double maxAgeSeconds) {
    return latestObservation != null && (Timer.getFPGATimestamp() - lastHeartbeat) <= maxAgeSeconds;
  }

  /** Convenience wrappers for future commands. */
  public void setPipeline(int index) {
    LimelightHelpers.setPipelineIndex(Constants.LimelightConstants.CAMERA_NAME, index);
  }

  public void setLedEnabled(boolean enabled) {
    if (enabled) {
      LimelightHelpers.setLEDMode_ForceOn(Constants.LimelightConstants.CAMERA_NAME);
    } else {
      LimelightHelpers.setLEDMode_ForceOff(Constants.LimelightConstants.CAMERA_NAME);
    }
  }

  private PoseEstimate sampleEstimate() {
    Optional<Alliance> alliance = DriverStation.getAlliance();
    if (alliance.isPresent() && alliance.get() == Alliance.Red) {
      return LimelightHelpers.getBotPoseEstimate_wpiRed_MegaTag2(
          Constants.LimelightConstants.CAMERA_NAME);
    }
    return LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(
        Constants.LimelightConstants.CAMERA_NAME);
  }

  private boolean isEstimateUsable(PoseEstimate estimate) {
    if (estimate == null || estimate.pose == null) {
      return false;
    }
    if (estimate.tagCount < Constants.LimelightConstants.MIN_TAG_COUNT) {
      return false;
    }
    return true;
  }

  private double computeAverageAmbiguity(PoseEstimate estimate) {
    RawFiducial[] fiducials = estimate.rawFiducials;
    if (fiducials == null || fiducials.length == 0) {
      return 0.0;
    }
    double totalAmbiguity = 0.0;
    for (RawFiducial fiducial : fiducials) {
      totalAmbiguity += fiducial.ambiguity;
    }
    return totalAmbiguity / fiducials.length;
  }

  private MeasurementNoise estimateNoise(PoseEstimate estimate) {
    double tagCountFactor = Math.max(1.0, estimate.tagCount);
    double distanceFactor =
        estimate.avgTagDist <= 0.0
            ? 1.0
            : Math.max(
                1.0,
                estimate.avgTagDist / Constants.LimelightConstants.DISTANCE_TRUST_FALLOFF_METERS);

    double xyStd =
        Constants.LimelightConstants.SINGLE_TAG_XY_STDDEV
            * distanceFactor
            / Math.min(3.0, tagCountFactor);
    double thetaStd =
        Constants.LimelightConstants.SINGLE_TAG_THETA_STDDEV
            * distanceFactor
            / Math.min(2.5, tagCountFactor);

    xyStd = Math.max(Constants.LimelightConstants.MIN_XY_STDDEV, xyStd);
    thetaStd = Math.max(Constants.LimelightConstants.MIN_THETA_STDDEV, thetaStd);

    return new MeasurementNoise(xyStd, thetaStd);
  }

  private record MeasurementNoise(double xyStdDev, double thetaStdDev) {}
}
