package frc.robot.tunerconstants;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import edu.wpi.first.units.measure.LinearVelocity;
import frc.robot.Constants.RuntimeConstants;

/** 
 * A helper class to manage swapping between TunerConstants for different robots.
 * To add new TunerConstants:
 * (1) create a file in the tunerconstants folder named "TunerConstants" + robotName with the pasted TunerConstants from Phoenix Tuner X.
 * (2) add to the {@link RobotTune} enum with the name of the robot and the class of the file you created in step 1.
 * (3) whenever you need to change robots, change {@link RuntimeConstants#CURRENT_ROBOT} to the corresponding RobotTune
 */
public class TunerConstantsHelper {
    /** 
     * An enum representing the robot being used. 
     * Instructions: {@link TunerConstantsHelper}
     */
  public enum RobotTune {
    PROGBOT(TunerConstantsProgbot.class); //Progbot can be used as an example

    private final Class<?> file;

    private RobotTune(Class<?> file) {
      this.file = file;
    }

    public Class<?> getFileClass() {
      return file;
    }
  }

  /**
   * Gets the static object from the current robot TunerConstants while catching exceptions
   * @param field the name of the static field
   * @return the Object from the field 
   */
  private static Object getStaticField(String field) {
    try {
      return RuntimeConstants.CURRENT_ROBOT.getFileClass().getField(field).get(null);
    } catch (Exception e) {
      throw new RuntimeException("Failed to load " + field + " from " + RuntimeConstants.CURRENT_ROBOT.name(), e);
    }
  }

  /* Initialize the TunerConstants fields */
  public static final CANBus kCANBus = (CANBus) getStaticField("kCanBus");

  public static final LinearVelocity kSpeedAt12Volts =
      (LinearVelocity) getStaticField("kSpeedAt12Volts");

  public static final SwerveDrivetrainConstants DrivetrainConstants =
      (SwerveDrivetrainConstants) getStaticField("DrivetrainConstants");

  @SuppressWarnings("unchecked") //doesn't like assertion of type SwerveModuleConstants<...> because it can't be implied
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
