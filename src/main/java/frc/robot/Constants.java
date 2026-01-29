package frc.robot;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.RobotBase;

public class Constants {
  /**
   * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when
   * running on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and
   * "replay" (log replay from a file).
   */
  public class RuntimeConstants {
    public static final Mode simMode = Mode.SIM;
    public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

    public static enum Mode {
      /** Running on a real robot. */
      REAL,

      /** Running a physics simulator. */
      SIM,

      /** Replaying from a log file. */
      REPLAY
    }
  }

  public final class ControllerConstants {
    // -- Port (identifier) for each individual joystick/controller.
    public static final int JOYSTICK_LEFT_PORT = 0;
    public static final int JOYSTICK_MIDDLE_PORT = 1;
    public static final int JOYSTICK_RIGHT_PORT = 2;

    /* -- Individual mappings for buttons on each joystick, from 1-16. Note that right-handed
     * -- controllers and left-handed controllers have opposite and reflected bindings (see below).
     */
    public static final int TRIGGER = 1;
    public static final int THUMB_BUTTON_BOTTOM = 2;
    public static final int THUMB_BUTTON_LEFT = 3;
    public static final int THUMB_BUTTON_RIGHT = 4;

    /* Each joystick is either left or right handed. When holding the joystick, the buttons on
     * the side of the opposing hand are the offhand buttons. The buttons on the side of the hand
     * holding the joystick is the mainhand buttons.
     */
    public static final int OFFHAND_TOP_LEFT = 5;
    public static final int OFFHAND_TOP_MIDDLE = 6;
    public static final int OFFHAND_TOP_RIGHT = 7;
    public static final int OFFHAND_BOTTOM_RIGHT = 8;
    public static final int OFFHAND_BOTTOM_MIDDLE = 9;
    public static final int OFFHAND_BOTTOM_LEFT = 10;

    public static final int MAINHAND_TOP_RIGHT = 11;
    public static final int MAINHAND_TOP_MIDDLE = 12;
    public static final int MAINHAND_TOP_LEFT = 13;
    public static final int MAINHAND_BOTTOM_LEFT = 14;
    public static final int MAINHAND_BOTTOM_MIDDLE = 15;
    public static final int MAINHAND_BOTTOM_RIGHT = 16;

    public static final double HEADING_DEADZONE = 0.2;
  }

  public final class DrivetrainConstants {
    public static final LinearVelocity MAX_LINEAR_VELOCITY = MetersPerSecond.of(1);
  }

  public static final class LimelightConstants {
    /** NetworkTables name configured on the camera. */
    public static final String CAMERA_NAME =
        "limelight";

    /**
     * Transform from the robot origin (center of rotation on the floor) to the camera pose. The
     * numbers below assume the camera sits 8 in forward, 0 in left/right, and 26 in above the floor
     * with a slight upward tilt; update once final mounting values are known.
     */
    public static final Transform3d ROBOT_TO_CAMERA =
        new Transform3d(
            new Translation3d(Inches.of(11.5), Inches.of(0.0), Inches.of(7.5)),
            new Rotation3d(0.0, Units.degreesToRadians(20.0), 0.0));
    // TODO: update to the final measured translation/rotation from robot origin to camera.

    /** Maximum pose ambiguity reported by Limelight to accept a measurement. */
    public static final double MAX_POSE_AMBIGUITY =
        0.2; // Lower value rejects noisy solves, raise if too many drops occur.
    /** Minimum number of tags contributing to the solve to accept a pose. */
    public static final int MIN_TAG_COUNT = 1; // Set to 2+ if single-tag jitter is problematic.

    /**
     * Baseline standard deviations (meters) for measurements that only use a single tag. Multi-tag
     * solves will scale these values down dynamically.
     */
    public static final double SINGLE_TAG_XY_STDDEV =
        0.45; // Reduce if the camera is steady on good tags.

    public static final double SINGLE_TAG_THETA_STDDEV =
        Units.degreesToRadians(7.5); // Adjust if heading noise is smaller/bigger.
    /** Floor for the best possible standard deviation. */
    public static final double MIN_XY_STDDEV = 0.05; // Safety floor for downstream pose estimators.

    public static final double MIN_THETA_STDDEV =
        Units.degreesToRadians(2.0); // Prevent unrealistically low heading noise.

    /** Distance where we start to inflate std devs significantly (meters). */
    public static final double DISTANCE_TRUST_FALLOFF_METERS =
        4.5; // Tune to how vision accuracy drops off with range.
  }

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
}
