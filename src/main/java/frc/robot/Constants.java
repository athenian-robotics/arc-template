package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Voltage;
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

  public final class OuttakeConstants {
    public static final int RIGHT_SHOOTER_MOTOR = -1;
    public static final int LEFT_SHOOTER_MOTOR = -1;
    public static final int INTAKE_MOTOR = -1;
    public static final int ANGLE_CHANGER_MOTOR = -1;

    public static final double INTAKE_TO_SHOOTER = 2;
    public static final double INTAKE_TO_GROUND = 2;

    public static final double MINIMUM_SHOT_ANGLE_DEG = 0.0;
    public static final double MAXIMUM_SHOT_ANGLE_DEG = 0.0;
    public static final double STARTING_SHOT_ANGLE_DEG = 0.0;
    public static final double MIDFIELD_SHOT_ANGLE_DEG = 0.0;
    public static final double OPPOSITE_TEAM_SHOT_ANGLE_DEG = 0.0;

    public static final double MIDFIELD_LIMIT_FEET = 0.0;
    public static final double OPPOSITE_TEAM_LIMIT_FEET = 0.0;

    public static final Translation2d HUB_POSITION = new Translation2d();
  }
}
