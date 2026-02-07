package frc.robot.subsystems.pathgeneration;

import org.littletonrobotics.junction.AutoLog;

public interface PathGenerationIO {
  @AutoLog
  public static class PathGenerationIOInputs {
    // Add inputs here if needed in the future
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(PathGenerationIOInputs inputs) {}
}
