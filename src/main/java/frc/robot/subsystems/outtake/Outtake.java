package frc.robot.subsystems.outtake;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.OuttakeConstants;
import org.littletonrobotics.junction.Logger;

public class Outtake extends SubsystemBase {
  private final OuttakeIO io;
  private final OuttakeIOInputsAutoLogged inputs;

  public Outtake() {
    io = new OuttakeIOTalonFX();
    inputs = new OuttakeIOInputsAutoLogged();
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Outtake", inputs);
  }

  /**
   * @return Returns a StartEndCommand that starts spinning the flywheel and stops spinning the
   *     flywheel when the command ends.
   */
  public Command enterShootMode(Translation2d currentPosition) {
    return new InstantCommand(io::startFlywheel).andThen(new InstantCommand(() -> io.setAngleAtTarget(currentPosition)));
  }

  /**
   * Starts the flywheel and angles the hood according to the parameter
   * @param shotAngleDeg The angle at which the ball will exit the shooter ccw+ from horizontal
   * @return The command
   */
  public Command enterShootMode(double shotAngleDeg) {
    return new InstantCommand(io::startFlywheel).andThen(new InstantCommand(() -> io.setAngle(shotAngleDeg)));
  }

  public Command stopFlywheel() {
    return new InstantCommand(io::stopFlywheel);
  }

  public Command sendBallsToShooter() {
    return new StartEndCommand(
        () -> io.setIntakeVoltage(OuttakeConstants.INTAKE_TO_SHOOTER),
        () -> io.setIntakeVoltage(0),
        this);
  }

  public Command groundOuttake() {
    return new StartEndCommand(
        () -> io.setIntakeVoltage(OuttakeConstants.INTAKE_TO_GROUND),
        () -> io.setIntakeVoltage(0),
        this);
  }

  public Command setHoodAngle(Angle angle) {
    return new InstantCommand();
  }
}
