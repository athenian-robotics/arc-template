package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.PoseEstimate;
import frc.robot.LimelightHelpers.RawFiducial;
import java.util.Optional;

public class VisionIOLimelight implements VisionIO {

  public VisionIOLimelight() {
    Translation3d translation = Constants.LimelightConstants.ROBOT_TO_CAMERA.getTranslation();
    Rotation3d rotation = Constants.LimelightConstants.ROBOT_TO_CAMERA.getRotation();
    LimelightHelpers.setCameraPose_RobotSpace(
        Constants.LimelightConstants.CAMERA_NAME,
        translation.getX(),
        translation.getY(),
        translation.getZ(),
        Units.radiansToDegrees(rotation.getX()),
        Units.radiansToDegrees(rotation.getY()),
        Units.radiansToDegrees(rotation.getZ()));
    LimelightHelpers.setPipelineIndex(Constants.LimelightConstants.CAMERA_NAME, 0);
  }

  /** Samples the Limelight network table to populate the loggable inputs. */
  @Override
  public void updateInputs(VisionIOInputs inputs) {
    inputs.hasTarget = LimelightHelpers.getTV(Constants.LimelightConstants.CAMERA_NAME);
    inputs.heartbeat =
        LimelightHelpers.getLimelightNTDouble(Constants.LimelightConstants.CAMERA_NAME, "hb");

    PoseEstimate estimate = sampleEstimate();
    if (estimate == null || estimate.pose == null) {
      inputs.hasEstimate = false;
      return;
    }

    inputs.hasEstimate = true;
    inputs.pose = estimate.pose;
    inputs.timestamp = estimate.timestampSeconds;
    inputs.tagCount = estimate.tagCount;
    inputs.avgTagDist = estimate.avgTagDist;
    inputs.avgTagArea = estimate.avgTagArea;
    inputs.isMegaTag2 = estimate.isMegaTag2;

    inputs.avgAmbiguity = computeAverageAmbiguity(estimate);

    MeasurementNoise noise = estimateNoise(estimate);
    inputs.xyStdDev = noise.xyStdDev;
    inputs.thetaStdDev = noise.thetaStdDev;
  }

  /**
   * Switches the vision pipeline.
   *
   * <p>This is provided so commands can change the Limelight configuration.
   */
  // Convenience wrappers for future commands.
  @Override
  public void setPipeline(int index) {
    LimelightHelpers.setPipelineIndex(Constants.LimelightConstants.CAMERA_NAME, index);
  }

  /** Toggles the Limelight LEDs. */
  @Override
  public void setLedEnabled(boolean enabled) {
    if (enabled) {
      LimelightHelpers.setLEDMode_ForceOn(Constants.LimelightConstants.CAMERA_NAME);
    } else {
      LimelightHelpers.setLEDMode_ForceOff(Constants.LimelightConstants.CAMERA_NAME);
    }
  }

  /**
   * Synchronizes the Limelight with the current gyro orientation.
   *
   * @param yaw Robot heading (degrees).
   * @param yawRate Robot angular velocity (degrees/sec).
   */
  @Override
  public void setRobotOrientation(double yaw, double yawRate) {
    LimelightHelpers.SetRobotOrientation_NoFlush(
        Constants.LimelightConstants.CAMERA_NAME, yaw, yawRate, 0, 0, 0, 0);
  }

  /**
   * Samples the alliance-specific pose estimate from the Limelight.
   *
   * <p>MegaTag2 support can be toggled via {@code useMegaTag2}.
   */
  private PoseEstimate sampleEstimate() {
    Optional<Alliance> alliance = DriverStation.getAlliance();
    // Use MegaTag1 for stability if MegaTag2 is glitchy.
    // Switch to MegaTag2 once gyro sync is verified and stable.
    boolean useMegaTag2 = false;

    if (alliance.isPresent() && alliance.get() == Alliance.Red) {
      if (useMegaTag2) {
        return LimelightHelpers.getBotPoseEstimate_wpiRed_MegaTag2(
            Constants.LimelightConstants.CAMERA_NAME);
      } else {
        return LimelightHelpers.getBotPoseEstimate_wpiRed(Constants.LimelightConstants.CAMERA_NAME);
      }
    }
    if (useMegaTag2) {
      return LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(
          Constants.LimelightConstants.CAMERA_NAME);
    } else {
      return LimelightHelpers.getBotPoseEstimate_wpiBlue(Constants.LimelightConstants.CAMERA_NAME);
    }
  }

  /**
   * Returns the average ambiguity ratio from all fiducials contributing to a solve.
   *
   * @param estimate Raw pose estimate returned by the Limelight API.
   */
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

  /**
   * Produces an estimated XY/theta noise envelope for reported poses.
   *
   * @param estimate Pose estimate returned by the Limelight library.
   */
  private MeasurementNoise estimateNoise(PoseEstimate estimate) {
    double tagCountFactor = Math.max(1.0, estimate.tagCount);
    double distanceFactor =
        Math.max(1.0, estimate.avgTagDist / Constants.LimelightConstants.DISTANCE_TRUST_FALLOFF_METERS);

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
