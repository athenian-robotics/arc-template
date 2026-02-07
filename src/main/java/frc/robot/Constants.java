package frc.robot;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
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
    public static final double MAX_LINEAR_VELOCITY = 1;
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

  public final class HopperConstants {
    public final static int SPARK_ID = 0;

    public final static double HOPPER_RETRACTED = 0;
    public final static double HOPPER_PARTIAL = 4;
    public final static double HOPPER_FULL = 11.425;

    public final static double HOPPER_WINCH_GEAR_RATIO = 10;
    public final static double HOPPER_WINCH_CIRCUMFRENCE = 0.75 * Math.PI;
    public final static double HOPPER_POSITION_TO_ANGLE_CONVERSION = HOPPER_WINCH_GEAR_RATIO / HOPPER_WINCH_CIRCUMFRENCE;
   
  
//feedback constants
    public final static double HOPPER_kP = 0;
    public final static double HOPPER_kI = 0;
    public final static double HOPPER_kD = 0;
//feedforward constants
    public final static double HOPPER_kS = 0;
    public final static double HOPPER_kV = 0;
    public final static double HOPPER_kA = 0;
//motion profiler constants
    public final static double HOPPER_CRUISE_VELOCITY = 0;
    public final static double HOPPER_MAX_ACCELERATION = 0;
    public final static double HOPPER_MAX_ALLOWED_PROFILER_ERROR = 0;

  }
  public final class IntakeConstants {
    
    public final static double kS = 0;
    public final static double kV = 0;

  }
}
