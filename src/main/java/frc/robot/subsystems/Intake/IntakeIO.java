package frc.robot.subsystems.intake;

import frc.robot.Constants.IntakeConstants;
import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
  @AutoLog
  public static class IntakeIOInputs {
    public double armMotor_Voltage_Volts = 0;
    public double armMotor_Current_Amps = 0;
    public double armMotorRotations_Rotations = 0;
    public double armRotations_Rotations =
        armMotorRotations_Rotations * IntakeConstants.GEAR_ROTATIONS_TO_ARM_ROTATIONS;
  }

  public void updateInputs(IntakeIOInputs inputs);
  public void goToPosition(double rotations);
}
