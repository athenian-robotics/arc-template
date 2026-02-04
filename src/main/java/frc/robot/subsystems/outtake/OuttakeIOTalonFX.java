package frc.robot.subsystems.outtake;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Feet;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.OuttakeConstants;

public class OuttakeIOTalonFX extends SubsystemBase implements OuttakeIO {
  private final TalonFX rightShooter, leftShooter, middleWheel, starWheel, angleChanger;
  private final OuttakeIOInputs logs;
  private double targetShotAngleDeg = OuttakeConstants.STARTING_SHOT_ANGLE_DEG;

  private double currentAngleDeg = 0.0;
  private double currentAngularVelocityDegPerSecond = 0.0;

  public OuttakeIOTalonFX() {
    super();
    logs = new OuttakeIOInputs();
    leftShooter = new TalonFX(OuttakeConstants.LEFT_SHOOTER_MOTOR);
    rightShooter = new TalonFX(OuttakeConstants.RIGHT_SHOOTER_MOTOR);

    leftShooter.setControl(
        new Follower(OuttakeConstants.RIGHT_SHOOTER_MOTOR, MotorAlignmentValue.Opposed));
    
    rightShooter.getConfigurator().apply(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Coast)); // To make re-spinning up faster
    middleWheel = new TalonFX(OuttakeConstants.MIDDLE_WHEEL_MOTOR);
    starWheel = new TalonFX(OuttakeConstants.STAR_WHEEL_MOTOR);
    angleChanger = new TalonFX(OuttakeConstants.ANGLE_CHANGER_MOTOR);
  }

  public void periodic() {
    currentAngleDeg = angleChanger.getPosition().getValue().in(Degrees) * OuttakeConstants.ANGLE_CHANGER_GEAR_RATIO;
    currentAngularVelocityDegPerSecond = angleChanger.getVelocity().getValue().in(DegreesPerSecond) * OuttakeConstants.ANGLE_CHANGER_GEAR_RATIO;

    // I dont know what I'm doing! What is a PID?!
    double angleError = targetShotAngleDeg - currentAngleDeg;
    double angularVelocityError = 0 - currentAngularVelocityDegPerSecond;

    double updatedAngle = currentAngleDeg + (angleError * OuttakeConstants.HOOD_ANGLE_KP);
    double updatedAngularVelocity = currentAngularVelocityDegPerSecond + (angularVelocityError * OuttakeConstants.HOOD_ANGLE_KD);

    angleChanger.set(updatedAngle);
  }

  /**
   * Calculates how to aim for the hub.
   *
   * @return the angle required to shoot into the hub from its location
   */
  public double calculateAngle() {
    // PUT INTO DESMOS V V V
    // a_{ngle}=\arctan\left(\frac{\left(v^{2}+\sqrt{v^{4}-g\left(gd^{2}+2hv^{2}\right)}\right)}{gd}\right)
    throw new UnsupportedOperationException("Unimplemented method 'calculateAngle'");
  }

  @Override
  public void updateInputs(OuttakeIOInputs logs) {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'updateLogs'");
  }

  public void startFlywheel() {
    rightShooter.setControl(new VoltageOut(12));
  }

  public void stopFlywheel() {
    rightShooter.set(0.0);
  }

  public void setMiddleWheelVoltage(double voltage) {
    middleWheel.setControl(new VoltageOut(voltage));
  }

  public void setStarWheelVoltage(double voltage) {
    starWheel.setControl(new VoltageOut(voltage));
  }

  public void setAngleAtTarget(Translation2d currentPosition) {
    if (currentPosition.getMeasureX().in(Feet) > OuttakeConstants.OPPOSITE_TEAM_LIMIT_FEET) {
      targetShotAngleDeg = OuttakeConstants.OPPOSITE_TEAM_SHOT_ANGLE_DEG;
      return;
    } 
    if (currentPosition.getMeasureX().in(Feet) > OuttakeConstants.MIDFIELD_LIMIT_FEET) {
      targetShotAngleDeg = OuttakeConstants.MIDFIELD_SHOT_ANGLE_DEG;
      return;
    }
    double shotDistanceFeet = currentPosition.getDistance(OuttakeConstants.HUB_POSITION);
    throw new UnsupportedOperationException("We haven't yet figured out the shot angle formula");
  }

  public void setAngle(double angleDegrees) {
    targetShotAngleDeg = angleDegrees;
  }
}
