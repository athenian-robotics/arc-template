package frc.robot.subsystems.outtake;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Feet;

import java.util.OptionalDouble;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

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

    // This represents the starting position of the hood
    angleChanger.setPosition(OuttakeConstants.ANGLE_CHANGER_STARTING_ANGLE_ROTATIONS / OuttakeConstants.ANGLE_CHANGER_GEAR_RATIO);
    currentAngleDeg = OuttakeConstants.ANGLE_CHANGER_STARTING_ANGLE_ROTATIONS * 360;
  }

  public void periodic() {
    currentAngleDeg = angleChanger.getPosition().getValue().in(Degrees) * OuttakeConstants.ANGLE_CHANGER_GEAR_RATIO;
    currentAngularVelocityDegPerSecond = angleChanger.getVelocity().getValue().in(DegreesPerSecond) * OuttakeConstants.ANGLE_CHANGER_GEAR_RATIO;

    double angleError = targetShotAngleDeg - currentAngleDeg;
    double angularVelocityError = 0 - currentAngularVelocityDegPerSecond;

    double appliedVoltage = (angleError * OuttakeConstants.HOOD_ANGLE_KP) + (angularVelocityError * OuttakeConstants.HOOD_ANGLE_KD);
    Logger.recordOutput("Outtake/AngleVoltage", appliedVoltage);

    angleChanger.setControl(new VoltageOut(appliedVoltage));
  }

  /**
   * Calculates how to aim for the hub.
   *
   * @return the angle required to shoot into the hub from its location
   */
  public OptionalDouble calculateAngle(Translation2d targetPosition) {
    // PASTE INTO DESMOS V V V
    // a_{ngle}=\arctan\left(\frac{\left(v^{2}+\sqrt{v^{4}-g\left(gd^{2}+2hv^{2}\right)}\right)}{gd}\right)
    // This is unreadable right now so use this ^ ^ ^

    // The height offset from the robot hood to the target in meters because metric is better
    double height = (OuttakeConstants.HUB_HEIGHT_FEET - OuttakeConstants.LAUNCH_HEIGHT_FEET) * 0.3048;
    // The outtake velocity it meters per second
    double velocity = OuttakeConstants.OUTTAKE_VELOCITY_MPS;
    // The gravitational constant of 9.8 m/s^2
    double gravity = OuttakeConstants.GRAVITATIONAL_CONSTANT_MPS2;
    // The horizontal part of the distance between the robot and the target
    double distance = 1;

    // If distance is zero there will (might?) be a division by zero
    if (distance == 0) return OptionalDouble.empty();
    
    // ~Math~

    // The discriminant is useful for telling how many solutions there are
    double discriminant = 
      Math.pow(velocity, 4) 
      - ((Math.pow(gravity, 2) * Math.pow(distance, 2))
      + (2 * gravity * height * Math.pow(velocity, 2))
    );
    
    // If discriminant isn't squareroot-able, then there are no solutions,
    // meaning no shot is possible. 
    if (discriminant < 0) return OptionalDouble.empty(); 

    // The rest of the math
    double angle = Math.atan(
      (Math.pow(velocity, 2) + Math.sqrt(discriminant)) 
      / (gravity * distance)
    );

    return OptionalDouble.of(angle);
  }

  @Override
  public void updateInputs(OuttakeIOInputs inputs) {
    inputs.currentShotAngleDegrees = currentAngleDeg;
    inputs.currentAngularVelocityDegPerSecond = currentAngularVelocityDegPerSecond;
    inputs.targetShotAngleDegrees = targetShotAngleDeg;
  }

  public void startFlywheel() {
    rightShooter.setControl(new VoltageOut(OuttakeConstants.FLYWHEEL_VOLTS));
    Logger.recordOutput("Outtake/FlywheelVoltage", OuttakeConstants.FLYWHEEL_VOLTS);
  }

  public void stopFlywheel() {
    rightShooter.set(0.0);
    Logger.recordOutput("Outtake/FlywheelVoltage", 0.0);
  }

  public void setMiddleWheelVoltage(double voltage) {
    middleWheel.setControl(new VoltageOut(voltage));
    Logger.recordOutput("Outtake/MiddleWheelVoltage", voltage);
  }

  public void setStarWheelVoltage(double voltage) {
    starWheel.setControl(new VoltageOut(voltage));
    Logger.recordOutput("Outtake/StarWheelVoltage", voltage);
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
