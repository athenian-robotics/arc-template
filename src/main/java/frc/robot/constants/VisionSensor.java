package frc.robot.constants;

import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

public final class VisionSensor {
    /*-----Tune these values for your robot-----*/

    /** Transform from the robot origin (center of rotation on the floor) to the camera position and heading. */
    public static final Transform3d ROBOT_TO_CAMERA =
        new Transform3d(
            new Translation3d(
                Inches.of(0.0), //forward positive
                Inches.of(0.0), //right positive
                Inches.of(0.0)), //up positive
            new Rotation3d(
                0.0, 
                Units.degreesToRadians(0.0), //pitch/vertical angle
                0.0));


    /*-----The values below should be relatively constant - can be reused with the camera-----*/

    /** NetworkTables name configured on the camera. */
    public static final String NAME = "limelight";

    /** 
     * Maximum pose ambiguity to accept a measurement. Lower values reject noisy solves.
     * Increased values drop readings more often.
     */
    public static final double MAX_POSE_AMBIGUITY = 0.2;

    /** 
     * Minimum number of tags contributing to the solve to accept a pose. 
     * A value of 2+ helps with single-tag jitter.
     */
    public static final int MIN_TAG_COUNT = 1;

    /**
     * Baseline standard deviations <b><i>(m)</i></b> for measurements that only use a single tag. Multi-tag solves
     * will scale these values down dynamically. Low values are fine if the camera is steady on good tags.
     */
    public static final double SINGLE_TAG_XY_STDDEV = 0.45;

    /**
     * Baseline standard deviations <b><i>(radians)</i></b> for measurements that only use a single tag. Multi-tag
     * solves will scale these values down dynamically. Adjusted based on heading noise.
     */
    public static final double SINGLE_TAG_THETA_STDDEV = Units.degreesToRadians(7.5);

    /** 
     * Safety floor for the best possible standard deviation, used by downstream pose estimators. 
     * <b><i>(m)</i></b> 
     */
    public static final double MIN_XY_STDDEV = 0.05; 

    /** 
     * Floor for the best possible standard deviation <b><i>(radians)</i></b>. 
     * Prevents unrealistically low heading noise. 
     */
    public static final double MIN_THETA_STDDEV = Units.degreesToRadians(2.0);

    /** 
     * Distance where standard deviations start to inflate significantly <b><i>(m)</i></b>. 
     * Tuned to how vision accuracy drops off with range. 
     */
    public static final double DISTANCE_TRUST_FALLOFF = 4.5;

    /** 
     * Maximum allowed pose translation error between vision and odometry before rejecting data <b><i>(m)</i></b>. 
     */
    public static final double MAX_TRANSLATION_ERROR = 2.0;

    /**
     * Maximum allowed rotation error <b><i>(radians)</i></b> between vision and odometry before rejecting data.
     */
    public static final double MAX_ROTATION_ERROR = Units.degreesToRadians(30.0);

    /** 
     * The maximum time before an observation is no longer considered fresh <b><i>(s)</i></b>. 
     */
    public static final double FRESH_OBSERVATION_THRESHOLD = 0.5;
}
