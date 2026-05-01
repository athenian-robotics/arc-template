package frc.robot.subsystems.intake;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.util.Units;
import frc.robot.Constants.IntakeConstants;

public class IntakeIOTalonFX implements IntakeIO {
  private final TalonFX armMotor = new TalonFX(IntakeConstants.TALON_ID);


  public IntakeIOTalonFX(){
    var talonFXConfigs = new TalonFXConfiguration();
    var slot0Configs = talonFXConfigs.Slot0;
    slot0Configs.kP = IntakeConstants.INTAKE_kP;
    slot0Configs.kI = IntakeConstants.INTAKE_kI;
    slot0Configs.kD = IntakeConstants.INTAKE_kD;
    slot0Configs.kS = IntakeConstants.INTAKE_kS;
    slot0Configs.kV = IntakeConstants.INTAKE_kV;
    
    var motionMagicConfigs = talonFXConfigs.MotionMagic;
    motionMagicConfigs.MotionMagicCruiseVelocity = 0;
    motionMagicConfigs.MotionMagicAcceleration = 0;
    motionMagicConfigs.MotionMagicJerk = 0;
    
    armMotor.getConfigurator().apply(talonFXConfigs);
  }

  /** Update the set of loggable inputs */
  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    inputs.armMotor_Voltage_Volts = (armMotor.getMotorVoltage().getValueAsDouble());
    inputs.armMotor_Current_Amps = armMotor.getTorqueCurrent().getValueAsDouble();
    inputs.armMotorRotations_Rotations = armMotor.getPosition().getValueAsDouble();
    inputs.armRotations_Rotations = 
        inputs.armMotorRotations_Rotations * IntakeConstants.GEAR_ROTATIONS_TO_ARM_ROTATIONS;
  }

  @Override
  public void goToPosition(double armRotations_Degrees) {
    final MotionMagicVoltage m_request = new MotionMagicVoltage(0);
    armMotor.setControl(m_request.withPosition(Units.degreesToRotations(armRotations_Degrees)  / IntakeConstants.GEAR_ROTATIONS_TO_ARM_ROTATIONS));
  }

}
