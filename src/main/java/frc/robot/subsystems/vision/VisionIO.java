package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import org.littletonrobotics.junction.AutoLog;

public interface VisionIO {
  @AutoLog
  public static class VisionIOInputs {
    public boolean hasTarget = false;
    public boolean hasEstimate = false;
    public int tagCount = 0;
    public double avgTagDist = 0.0;
    public double avgTagArea = 0.0;
    public double avgAmbiguity = 0.0;
    public Pose2d pose = new Pose2d();
    public double timestamp = 0.0;
    public boolean isMegaTag2 = false;
    public double heartbeat = 0.0;
    public double xyStdDev = 0.0;
    public double thetaStdDev = 0.0;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(VisionIOInputs inputs) {}

  /** Sets the pipeline index. */
  public default void setPipeline(int index) {}

  /** Sets the LED mode. */
  public default void setLedEnabled(boolean enabled) {}

  /** Sets the robot orientation for MegaTag2. */
  public default void setRobotOrientation(double yaw, double yawRate) {}
}
