package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.Optional;

/** The subsystem for handling vision. */
public class Vision extends SubsystemBase {
  private static final double MAX_TRANSLATION_ERROR_METERS = 2.0;
  private static final double MAX_ROTATION_ERROR_RADIANS = Units.degreesToRadians(30.0);

  private final LimelightSubsystem limelightSubsystem;

  public Vision(LimelightSubsystem limelightSubsystem) {
    this.limelightSubsystem = limelightSubsystem;
  }

  /**
   * @param estimatedPose The current position, according to odometry. May be innacurate.
   * @return Either a vision estimate of robot position from the vision sensor, or a None value if
   *     no estimate can be made.
   */
  public Optional<Pose2d> findRobotPose(Pose2d estimatedPose) {
    if (!limelightSubsystem.hasFreshObservation(0.5)) {
      return Optional.empty();
    }

    return limelightSubsystem
        .getLatestObservation()
        .filter(observation -> measurementMatchesOdometry(estimatedPose, observation.pose()))
        .map(LimelightSubsystem.VisionObservation::pose);
  }

  private boolean measurementMatchesOdometry(Pose2d reference, Pose2d measurement) {
    Translation2d delta = reference.getTranslation().minus(measurement.getTranslation());
    Rotation2d rotationDelta = reference.getRotation().minus(measurement.getRotation());
    return delta.getNorm() <= MAX_TRANSLATION_ERROR_METERS
        && Math.abs(rotationDelta.getRadians()) <= MAX_ROTATION_ERROR_RADIANS;
  }

  @Override
  public void periodic() {
    // LimelightSubsystem runs its own periodic loop; this wrapper simply offers convenient APIs.
  }
}
