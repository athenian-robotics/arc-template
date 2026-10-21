package frc.robot.constants;

import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.tunerconstants.TunerConstantsHelper.RobotTune;

public final class RobotConstants {
    /** The {@link RobotTune} for the current robot */
    public static final RobotTune CURRENT_ROBOT = RobotTune.PROGBOT;

    /** Mass of the robot with the battery <b><i>(kg)</b></i> */
    public static final double MASS = 74.088;
    /** The moment of inertia for the robot <b><i>(kg &times m^2)</b></i> */
    public static final double MOI = 6.883;
    /** The coefficient of friction between the wheels and the floor */
    public static final double WHEEL_COF = 1.2;

    /**
     * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when
     * running on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and
     * "replay" (log replay from a file).
     */
    public static class RuntimeConstants {
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
}
