package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.Optional;

// The library is kept in package `frc.robot` because git reasons. The library is in
// src/lib/LimelightHelpers

/** The subsystem for handling vision. */
public class Vision extends SubsystemBase {
  private static final double MAX_TRANSLATION_ERROR_METERS = 2.0;
  private static final double MAX_ROTATION_ERROR_RADIANS = Units.degreesToRadians(30.0);

  private final Limelight limelight;

  public Vision(Limelight limelight) {
    this.limelight = limelight;
  }

  /**
   * Returns the latest vision observation, optionally filtered by a pose estimate.
   *
   * @param currentPose The current pose estimate to use for filtering. If null, no filtering is
   *     applied (useful for seeding).
   * @return The latest vision observation if valid and (optionally) matches the estimate.
   */
  public Optional<Limelight.VisionObservation> getVisionObservation(Pose2d currentPose) {
    if (!limelight.hasFreshObservation(0.5)) {
      return Optional.empty();
    }

    return limelight
        .getLatestObservation()
        .filter(
            observation -> {
              if (currentPose == null) {
                return true;
              }
              return measurementMatchesOdometry(currentPose, observation.pose());
            });
  }

  public void setRobotOrientation(Rotation2d rotation, double yawVelocityRadPerSec) {
    limelight.setRobotOrientation(
        rotation.getDegrees(), Units.radiansToDegrees(yawVelocityRadPerSec));
  }

  private boolean measurementMatchesOdometry(Pose2d reference, Pose2d measurement) {
    Translation2d delta = reference.getTranslation().minus(measurement.getTranslation());
    Rotation2d rotationDelta = reference.getRotation().minus(measurement.getRotation());
    return delta.getNorm() <= MAX_TRANSLATION_ERROR_METERS
        && Math.abs(rotationDelta.getRadians()) <= MAX_ROTATION_ERROR_RADIANS;
  }

  @Override
  public void periodic() {
    // TODO Auto-generated method stub
  }
}
