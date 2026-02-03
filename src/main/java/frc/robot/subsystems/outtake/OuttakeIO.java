package frc.robot.subsystems.outtake;

import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Subsystem;
import org.littletonrobotics.junction.AutoLog;

public interface OuttakeIO extends Subsystem {
  @AutoLog
  public static class OuttakeIOInputs {
    /** The current speed in rpm of the outtake motor */
    public double outtakeMotorSpeed = 0.0;
    /** The set voltage of the outtake motor */
    public double outtakeMotorVoltage = 0.0;
    /** The actual angle of the hood, in degrees, representing launch angle compared to horizontal with ccw+ */
    public double currentHoodAngleDegrees = 0.0;
    /** The target angle of the hood, following the above rules */
    public double targetHoodAngleDegrees = 0.0;
  }

  /** Updates logs; util for AdvantageScope
   * @param inputs inputs from previous iteration
   */
  public void updateInputs(OuttakeIOInputs inputs);

  /** Causes the flywheel to start spinning up */
  public void startFlywheel();

  /** Causes the flywheel to coast */
  public void stopFlywheel();

  /** Causes the "intake" wheel to spin using specified voltage */
  public void setIntakeVoltage(double voltage);

  /** Sets the target hood angle, which the hood will constantly move towards */
  public void setTargetHoodAngle(double angleDegrees);
}
