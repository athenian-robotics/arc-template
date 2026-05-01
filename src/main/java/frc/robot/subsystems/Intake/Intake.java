package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.intake.IntakeIOInputsAutoLogged;

import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {

  private IntakeIO io;
  private IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Intake", inputs);
  }

  public void fullyExtend() {
    io.goToPosition(IntakeConstants.FULL_EXTENSION_DEGREES); // position feedback loop
  }
  public void fullyRetract(){
    io.goToPosition(IntakeConstants.FULL_RETRACTION_DEGREES);
  }

  public void wiggleUp() {
    io.goToPosition(IntakeConstants.MAX_WIGGLE_DEGREES);
     // velocity feedback loop
  }
}
