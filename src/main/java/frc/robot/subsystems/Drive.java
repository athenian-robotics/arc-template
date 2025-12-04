package frc.robot.subsystems;

import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DrivetrainConstants;
import java.io.File;
import swervelib.SwerveDrive;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;

public class Drive extends SubsystemBase {
  private final SwerveDrive drivetrain;

  private static final File cfgPath = new File(Filesystem.getDeployDirectory(), "swerve");

  // -- Alerts --

  public Drive() {
    try {
      drivetrain =
          new SwerveParser(cfgPath)
              .createSwerveDrive(DrivetrainConstants.MAX_LINEAR_VELOCITY.in(MetersPerSecond));
    } catch (Exception e) {
      throw new RuntimeException("Unable to load drive subsystem", e);
    }

    SwerveDriveTelemetry.verbosity = SwerveDriveTelemetry.TelemetryVerbosity.HIGH;
  }

  /**
   * Resets the odometry to the given pose.
   *
   * @param FieldPos The pose to reset the odometry to
   */
  public void resetOdometry(Pose2d fieldPos) {}
}
