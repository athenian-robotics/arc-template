package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.Optional;
import java.util.OptionalDouble;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.Constants.LimelightConstants;
import frc.robot.LimelightHelpers.RawFiducial;
import org.littletonrobotics.junction.Logger;

/** Subsystem shell that limits and validates pose updates from vision hardware. */
public class Vision extends SubsystemBase {
  private final VisionIO io;
  private final VisionIOInputsAutoLogged inputs = new VisionIOInputsAutoLogged();
  private VisionObservation latestObservation;
  private double lastHeartbeatSeconds = 0.0;
  private boolean overrideOdometry = true;
  private int sequentialRejections = 0;
  private int remainingOverrides = 0;

  /** Result of a validated Limelight solve. */
  public static record VisionObservation(
      /** Pose of the robot in field coordinates as computed by Limelight. */
      Pose2d pose,
      /** FPGA timestamp (seconds) captured with the snapshot. */
      double timestampSeconds,
      /** Estimated XY measurement noise (meters). */
      double xyStdDevMeters,
      /** Estimated yaw measurement noise (radians). */
      double thetaStdDevRad,
      /** Number of AprilTags used in the pose solve. */
      int tagCount,
      /** Average distance to the tags that contributed (meters). */
      double avgTagDistanceMeters,
      /** Average tag area as a percentage of the image. */
      double avgTagAreaPercent,
      /** Aggregate tracker ambiguity ratio (0-1). */
      double avgAmbiguityRatio,
      /** Whether the MegaTag2 solver produced the pose. */
      boolean isMegaTag2) {}

  /** Creates the vision subsystem backed by the supplied IO implementation. */
  public Vision(VisionIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Vision", inputs);

    if (inputs.hasEstimate && isEstimateUsable(inputs)) {
      latestObservation =
          new VisionObservation(
              inputs.pose,
              inputs.timestamp,
              inputs.xyStdDev,
              inputs.thetaStdDev,
              inputs.tagCount,
              inputs.avgTagDist,
              inputs.avgTagArea,
              inputs.avgAmbiguity,
              inputs.isMegaTag2);
      lastHeartbeatSeconds = Timer.getFPGATimestamp();
    }
  }

  /**
   * Returns the latest vision observation, optionally filtered by a pose estimate.
   *
   * @param currentPose The current pose estimate to use for filtering. If null, no filtering is
   *     applied (useful for seeding).
   * @return The latest vision observation if valid and (optionally) matches the estimate.
   */
  public Optional<VisionObservation> getVisionObservation(Pose2d currentPose) {
    if (!hasFreshObservation(Constants.LimelightConstants.FRESH_OBSERVATION_THRESHOLD)) {
      Logger.recordOutput("Vision/HasFreshObservation", false);
      sequentialRejections = 0;
      return Optional.empty();
    }
    Logger.recordOutput("Vision/HasFreshObservation", true);

    if (latestObservation == null) {
      sequentialRejections = 0;
      return Optional.empty();
    }

    Logger.recordOutput("Vision/SequentialRejections", sequentialRejections);

    boolean bypassOdometryCheck = overrideOdometry || currentPose == null 
      || sequentialRejections > LimelightConstants.MAX_SEQUENTIAL_REJECTIONS || remainingOverrides > 0;
    Logger.recordOutput("Vision/OdometryCheckBypassed", bypassOdometryCheck);

    if (!bypassOdometryCheck
        && !measurementMatchesOdometry(currentPose, latestObservation.pose())) {
      Logger.recordOutput("Vision/RejectedByOdometry", true);
      sequentialRejections += 1;
      return Optional.empty();
    } else if (bypassOdometryCheck) {
      remainingOverrides -= 1;
    }
    sequentialRejections = 0;
    Logger.recordOutput("Vision/RejectedByOdometry", false);

    // "Override" is intended as a one-shot bypass to allow relocalization when odometry is known
    // to be wrong (e.g., after a manual pose reset). However, pose estimation doesn't fully
    // apply a vision estimate that's far from its current estimate, so we need to override a few times.
    // After we accept a few measurements with the bypass enabled, 
    // re-enable normal gating to prevent large frame-mismatch teleports.
    if (overrideOdometry) {
      overrideOdometry = false;
      remainingOverrides = LimelightConstants.SEQUENTIAL_OVERRIDES;
    }

    return Optional.of(latestObservation);
  }

  /**
   * Updates the Limelight's notion of the robot orientation so that pose solutions remain
   * synchronized with the gyro.
   *
   * @param rotation the current accurate rotation
   * @param yawVelocityRadPerSec the current accurate (yaw) rotation speed
   */
  public void setRobotOrientation(Rotation2d rotation, double yawVelocityRadPerSec) {
    io.setRobotOrientation(rotation.getDegrees(), Units.radiansToDegrees(yawVelocityRadPerSec));
  }

  /**
   * Returns the reported distance from the robot to a specific AprilTag, if detected.
   *
   * @param tagNumber AprilTag ID to query.
   * @return Optional distance to the requested tag in meters.
   */
  public OptionalDouble getDistanceToTag(int tagNumber) {
    RawFiducial[] fiducials =
        LimelightHelpers.getRawFiducials(Constants.LimelightConstants.CAMERA_NAME);
    for (RawFiducial fiducial : fiducials) {
      if (fiducial.id == tagNumber) {
        return OptionalDouble.of(fiducial.distToRobot);
      }
    }
    return OptionalDouble.empty();
  }

  /** Returns true if the measurement falls within the permitted translation and rotation window. */
  private boolean measurementMatchesOdometry(Pose2d reference, Pose2d measurement) {
    Translation2d delta = reference.getTranslation().minus(measurement.getTranslation());
    Rotation2d rotationDelta = reference.getRotation().minus(measurement.getRotation());
    return delta.getNorm() <= Constants.LimelightConstants.MAX_TRANSLATION_ERROR_METERS
        && Math.abs(rotationDelta.getRadians())
            <= Constants.LimelightConstants.MAX_ROTATION_ERROR_RADIANS;
  }

  /** Determines whether a cached observation is still recent enough to use. */
  private boolean hasFreshObservation(double maxAgeSeconds) {
    double timestamp = Timer.getFPGATimestamp();
    Logger.recordOutput("Vision/ObservationFreshnessSec", timestamp - lastHeartbeatSeconds);
    return latestObservation != null
        && (timestamp - lastHeartbeatSeconds) <= maxAgeSeconds;
  }

  /**
   * Filters pose updates before caching them locally.
   *
   * @return true when the raw vision inputs meet the configured thresholds.
   */
  private boolean isEstimateUsable(VisionIOInputsAutoLogged inputs) {
    return (inputs.tagCount >= Constants.LimelightConstants.MIN_TAG_COUNT
        && inputs.avgAmbiguity <= Constants.LimelightConstants.MAX_POSE_AMBIGUITY);
  }

  public void setOverrideOdometry(boolean value) {
    overrideOdometry = value;
  }

  public boolean odometryBeingOverridden () {
    return overrideOdometry || sequentialRejections > 0;
  }
}