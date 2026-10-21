package frc.robot.constants;

public final class DriveCommandsConstants {
    public static final double DEADBAND = OperationConfig.HEADING_DEADBAND;
    public static final double ANGLE_KP = 20.0;
    public static final double ANGLE_KD = 0.4;
    /** <b><i>(rad/s)</i></b> */
    public static final double ANGLE_MAX_VELOCITY = 16.0;
    /** <b><i>(rad/s/s)</i></b> */
    public static final double ANGLE_MAX_ACCELERATION = 20.0;
    /** Feed forward start delay <b><i>(s)</i></b> */
    public static final double FF_START_DELAY = 2.0;
    /** Feed forward ramp rate <b><i>(V/s)</i></b> */
    public static final double FF_RAMP_RATE = 0.1;
    /** <b><i>(rad/s)</i></b> */
    public static final double WHEEL_RADIUS_MAX_VELOCITY = 0.25;
    /** <b><i>(rad/s/s)</i></b> */
    public static final double WHEEL_RADIUS_RAMP_RATE = 0.05;
}