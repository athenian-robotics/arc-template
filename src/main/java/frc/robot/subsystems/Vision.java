package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.Optional;

// The library is kept in package `frc.robot` because git reasons. The library is in
// src/lib/LimelightHelpers

/** The subsystem for handling vision. */
public class Vision extends SubsystemBase {
  /**
   * @param estimatedPose The current position, according to odometry. May be innacurate.
   * @return Either a vision estimate of robot position from the vision sensor, or a None value if
   *     no estimate can be made.
   */
  public Optional<Pose2d> findRobotPose(Pose2d estimatedPose) {
    // TODO please code me ):
    return Optional.empty();
  }

  @Override
  public void periodic() {
    // TODO Auto-generated method stub
  }
}
