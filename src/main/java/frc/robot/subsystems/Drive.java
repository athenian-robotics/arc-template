package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DrivetrainConstants;
import java.io.File;
import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;
import swervelib.SwerveDrive;
import swervelib.SwerveDriveTest;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;

public class Drive extends SubsystemBase {
  private final SwerveDrive drivetrain;

  @AutoLog
  public static class SwerveInputs {
    public Pose2d pose = new Pose2d();
    public Rotation2d odometryHeading = new Rotation2d();
  }

  private final SwerveInputsAutoLogged inputs = new SwerveInputsAutoLogged();

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

  public void driveFieldCentric(ChassisSpeeds velocity) {
    drivetrain.driveFieldOriented(velocity);
  }

  public void test() {
    SwerveDriveTest.powerDriveMotorsDutyCycle(drivetrain, 0.5);
  }

  public void driveRobotCentric(ChassisSpeeds velocity) {
    drivetrain.drive(velocity);
  }

  public void lockPose() {
    drivetrain.lockPose();
  }

  public ChassisSpeeds getChassisSpeeds() {
    return drivetrain.getFieldVelocity();
  }

  public void updateOdometryEstimate(Pose2d newPose, Time timestamp) {
    drivetrain.swerveDrivePoseEstimator.addVisionMeasurement(newPose, timestamp.in(Seconds));
  }

  @Override
  public void periodic() {
    updateLogInputs();
    Logger.processInputs(super.getName(), inputs);
  }

  private void updateLogInputs() {
    inputs.pose = drivetrain.getPose();
    inputs.odometryHeading = drivetrain.getOdometryHeading();
  }
}
