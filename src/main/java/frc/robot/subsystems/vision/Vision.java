package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;

/** The subsystem for handling vision. */
public class Vision extends SubsystemBase {
  private static final double MAX_TRANSLATION_ERROR_METERS = 2.0;
  private static final double MAX_ROTATION_ERROR_RADIANS = Units.degreesToRadians(30.0);

  private final VisionIO io;
  private final VisionIOInputsAutoLogged inputs = new VisionIOInputsAutoLogged();
  private VisionObservation latestObservation;
  private double lastHeartbeatSeconds = 0.0;

  public static record VisionObservation(
      Pose2d pose,
      double timestampSeconds,
      double xyStdDevMeters,
      double thetaStdDevRad,
      int tagCount,
      double avgTagDistanceMeters,
      double avgTagAreaPercent,
      double avgAmbiguityRatio,
      boolean isMegaTag2) {}

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
    if (!hasFreshObservation(0.5)) {
      Logger.recordOutput("Vision/HasFreshObservation", false);
      return Optional.empty();
    }
    Logger.recordOutput("Vision/HasFreshObservation", true);

    if (latestObservation == null) {
      return Optional.empty();
    }

    if (currentPose != null && !measurementMatchesOdometry(currentPose, latestObservation.pose())) {
      Logger.recordOutput("Vision/RejectedByOdometry", true);
      return Optional.empty();
    }
    Logger.recordOutput("Vision/RejectedByOdometry", false);

    return Optional.of(latestObservation);
  }

  public void setRobotOrientation(Rotation2d rotation, double yawVelocityRadPerSec) {
    io.setRobotOrientation(rotation.getDegrees(), Units.radiansToDegrees(yawVelocityRadPerSec));
  }

  private boolean measurementMatchesOdometry(Pose2d reference, Pose2d measurement) {
    Translation2d delta = reference.getTranslation().minus(measurement.getTranslation());
    Rotation2d rotationDelta = reference.getRotation().minus(measurement.getRotation());
    return delta.getNorm() <= MAX_TRANSLATION_ERROR_METERS
        && Math.abs(rotationDelta.getRadians()) <= MAX_ROTATION_ERROR_RADIANS;
  }

  private boolean hasFreshObservation(double maxAgeSeconds) {
    return latestObservation != null
        && (Timer.getFPGATimestamp() - lastHeartbeatSeconds) <= maxAgeSeconds;
  }
  private boolean isEstimateUsable(VisionIOInputsAutoLogged inputs) {
    if (inputs.tagCount < Constants.LimelightConstants.MIN_TAG_COUNT) {
      return false;
    }
    if (inputs.avgAmbiguity > Constants.LimelightConstants.MAX_POSE_AMBIGUITY) {
      return false;
    }
    return true;
  }
}
