package frc.robot.subsystems.outtake;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Subsystem;
import org.littletonrobotics.junction.AutoLog;

public interface OuttakeIO extends Subsystem {
  @AutoLog
  public static class OuttakeIOInputs {
    /** The actual angle of the hood, in degrees, representing launch angle compared to horizontal with ccw+ */
    public double currentShotAngleDegrees = 0.0;
    /** The target angle of the hood, in degrees, representing launch angle compared to horizontal with ccw+ */
    public double targetShotAngleDegrees = 0.0;
    /** The current angular velocity of the hood in degrees per second */
    public double currentAngularVelocityDegPerSecond = 0.0;
    /** Distance to the target in feet */
    public double targetDistanceFeet = 0.0;
  }

  /** Updates logs; util for AdvantageScope
   * @param inputs inputs from previous iteration
   */
  public void updateInputs(OuttakeIOInputs inputs);

  /** Causes the flywheel to start spinning up */
  public void startFlywheel();

  /** Causes the flywheel to coast */
  public void stopFlywheel();

  /** Causes the "middle" wheel to spin using specified voltage */
  public void setMiddleWheelVoltage(double voltage);

  /** Causes the star wheel to spin using specified voltage */
  public void setStarWheelVoltage(double voltage);

  /** Sets the target shot angle, which the hood will constantly move towards, to be targeting the target */
  public void setAngleAtTarget(Translation2d currentPosition);

  /** Sets the target shot angle, which the hood will constantly move towards, measured ccw+ from horiontal */
  public void setAngle(double angleDegrees);

  /** Sets the target angle based on the current network table value at /Outtake/HoodAngleDeg */
  public void setAngleFromNT ();
}
