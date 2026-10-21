package frc.robot.constants;

import static edu.wpi.first.units.Units.MetersPerSecond;

import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.tunerconstants.TunerConstantsHelper;

public final class PathGenerationConstants {
    // Predefined locations of interest
    public enum Location {
        TEST_POSE(new Pose2d(14.55, 1.0, new Rotation2d())); // Example coordinates

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
    /** <b><i>(m/s)</i></b> */
    public static final double MAX_VELOCITY = TunerConstantsHelper.kSpeedAt12Volts.in(MetersPerSecond);
    /** From PathPlanner <b><i>(m/s/s)</i></b> */
    public static final double MAX_ACCELERATION = 7.9;
    /** <b><i>(rad/s)</i></b> */
    public static final double MAX_ANGULAR_VELOCITY = MAX_VELOCITY / Math.sqrt(Math.pow(0.273, 2) * 2);
    /** From PathPlanner <b><i>(rad/s/s)</i></b> */
    public static final double MAX_ANGULAR_ACCELERATION = Units.degreesToRadians(790);

    public static final PathConstraints DEFAULT_CONSTRAINTS =
        new PathConstraints(
            MAX_VELOCITY,
            MAX_ACCELERATION,
            MAX_ANGULAR_VELOCITY,
            MAX_ANGULAR_ACCELERATION);
}
