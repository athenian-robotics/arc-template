package frc.robot.tunerconstants;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import edu.wpi.first.units.measure.LinearVelocity;

public class TunerConstantsHelper {
  enum Robot {
    FLOUNDER(TunerConstantsFlounder.class),
    PROGBOT(TunerConstantsProgbot.class);

    private final Class<?> file;

    private Robot(Class<?> file) {
      this.file = file;
    }

    public Class<?> getFileClass() {
      return file;
    }
  }

  /** The current robot, needs to be manually set at {@link TunerConstantsHelper#CURRENT_ROBOT} */
  public static final Robot CURRENT_ROBOT = Robot.FLOUNDER;

  private static Object getStaticField(String field) {
    try {
      return CURRENT_ROBOT.getFileClass().getField(field).get(null);
    } catch (Exception e) {
      throw new RuntimeException("Failed to load " + field + " from " + CURRENT_ROBOT.name(), e);
    }
  }

  public static final CANBus kCANBus = (CANBus) getStaticField("kCanBus");

  public static final LinearVelocity kSpeedAt12Volts =
      (LinearVelocity) getStaticField("kSpeedAt12Volts");

  public static final SwerveDrivetrainConstants DrivetrainConstants =
      (SwerveDrivetrainConstants) getStaticField("DrivetrainConstants");

  @SuppressWarnings("unchecked")
  public static final SwerveModuleConstants<
          TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
      FrontLeft =
          (SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>)
              getStaticField("FrontLeft");

  @SuppressWarnings("unchecked")
  public static final SwerveModuleConstants<
          TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
      FrontRight =
          (SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>)
              getStaticField("FrontRight");

  @SuppressWarnings("unchecked")
  public static final SwerveModuleConstants<
          TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
      BackLeft =
          (SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>)
              getStaticField("BackLeft");

  @SuppressWarnings("unchecked")
  public static final SwerveModuleConstants<
          TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
      BackRight =
          (SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>)
              getStaticField("BackRight");
}
