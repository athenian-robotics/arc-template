package frc.robot;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.MetersPerSecond;

import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.tunerconstants.TunerConstantsHelper;
import frc.robot.tunerconstants.TunerConstantsHelper.RobotTune;

public class Constants {
  

  public final class DriveCommandsConstants {
    public static final double DEADBAND = 0.1;
    public static final double ANGLE_KP = 20.0;
    public static final double ANGLE_KD = 0.4;
    public static final double ANGLE_MAX_VELOCITY = 16.0;
    public static final double ANGLE_MAX_ACCELERATION = 20.0;
    public static final double FF_START_DELAY = 2.0; // Secs
    public static final double FF_RAMP_RATE = 0.1; // Volts/Sec
    public static final double WHEEL_RADIUS_MAX_VELOCITY = 0.25; // Rad/Sec
    public static final double WHEEL_RADIUS_RAMP_RATE = 0.05; // Rad/Sec^2
  }

  public final class PathGenerationConstants {

    // Predefined locations of interest
    public enum Location {
      SPEAKER_CENTER(new Pose2d(0.0, 5.5, Rotation2d.fromDegrees(0))), // Example coordinates
      AMP(new Pose2d(1.8, 7.7, Rotation2d.fromDegrees(90))),
      SOURCE_RIGHT(new Pose2d(15.5, 1.0, Rotation2d.fromDegrees(120))),
      STAGE_CENTER(new Pose2d(4.5, 4.0, Rotation2d.fromDegrees(0))),
      TEST_POSE(new Pose2d(14.55, 1.0, new Rotation2d()));

      private final Pose2d pose;

      Location(Pose2d pose) {
        this.pose = pose;
      }

      public Pose2d getPose() {
        return pose;
      }
    }

    // Default constraints for pathfinding
    // Adjust these based on your robot's capabilities
    public static final double MAX_VELOCITY = TunerConstantsHelper.kSpeedAt12Volts.in(MetersPerSecond);
    public static final double MAX_ACCELERATION = 7.9;
    public static final double MAX_ANGULAR_VELOCITY =
        MAX_VELOCITY / Math.sqrt(Math.pow(0.273, 2) * 2);
    public static final double MAX_ANGULAR_ACCELERATION = Units.degreesToRadians(790);

    public static final PathConstraints DEFAULT_CONSTRAINTS =
        new PathConstraints(
            MAX_VELOCITY, // Max velocity (m/s)
            MAX_ACCELERATION, // Max acceleration (m/s^2) (from PathPlanner)
            MAX_ANGULAR_VELOCITY, // Max angular velocity (rad/s)
            MAX_ANGULAR_ACCELERATION); // Max angular acceleration (rad/s^2) (from PathPlanner)
  }
}
