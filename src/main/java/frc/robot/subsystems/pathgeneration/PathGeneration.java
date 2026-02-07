package frc.robot.subsystems.pathgeneration;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.HashMap;
import java.util.Map;

import org.littletonrobotics.junction.Logger;

public class PathGeneration extends SubsystemBase {
  private final PathGenerationIO io;
  private final PathGenerationIOInputsAutoLogged inputs = new PathGenerationIOInputsAutoLogged();

  // Default constraints for pathfinding
  // Adjust these based on your robot's capabilities
  private static final PathConstraints DEFAULT_CONSTRAINTS =
      new PathConstraints(
          3.0, // Max velocity (m/s)
          3.0, // Max acceleration (m/s^2)
          Units.degreesToRadians(540), // Max angular velocity (rad/s)
          Units.degreesToRadians(720)); // Max angular acceleration (rad/s^2)

  // Predefined locations of interest
  public enum Location {
    SPEAKER_CENTER(new Pose2d(0.0, 5.5, Rotation2d.fromDegrees(0))), // Example coordinates
    AMP(new Pose2d(1.8, 7.7, Rotation2d.fromDegrees(90))),
    SOURCE_RIGHT(new Pose2d(15.5, 1.0, Rotation2d.fromDegrees(120))),
    STAGE_CENTER(new Pose2d(4.5, 4.0, Rotation2d.fromDegrees(0)));

    private final Pose2d pose;

    Location(Pose2d pose) {
      this.pose = pose;
    }

    public Pose2d getPose() {
      return pose;
    }
  }

  private final Map<String, Pose2d> customLocations = new HashMap<>();

  public PathGeneration(PathGenerationIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("PathGeneration", inputs);
  }

  /**
   * Generates a command to pathfind to a specific known location.
   *
   * @param location The target location of interest.
   * @return The pathfinding command.
   */
  public Command pathfindTo(Location location) {
    return pathfindTo(location.getPose(), DEFAULT_CONSTRAINTS);
  }

  /**
   * Generates a command to pathfind to a specific known location with custom constraints.
   *
   * @param location The target location of interest.
   * @param constraints The path constraints to use.
   * @return The pathfinding command.
   */
  public Command pathfindTo(Location location, PathConstraints constraints) {
    return pathfindTo(location.getPose(), constraints);
  }

  /**
   * Generates a command to pathfind to an arbitrary pose.
   *
   * @param targetPose The target pose on the field.
   * @return The pathfinding command using default constraints.
   */
  public Command pathfindTo(Pose2d targetPose) {
    return pathfindTo(targetPose, DEFAULT_CONSTRAINTS);
  }

  /**
   * Generates a command to pathfind to an arbitrary pose with custom constraints.
   *
   * @param targetPose The target pose on the field.
   * @param constraints The path constraints to use.
   * @return The pathfinding command.
   */
  public Command pathfindTo(Pose2d targetPose, PathConstraints constraints) {
    // 0.0 is the goal end velocity (stop at the end)
    return AutoBuilder.pathfindToPose(targetPose, constraints, 0.0);
  }

  /**
   * Generates a command to pathfind to a location while running another command in parallel.
   * The parallel command will run until the pathfinding is complete.
   *
   * @param location The target location.
   * @param parallelCommand The command to run while moving.
   * @return A ParallelDeadlineGroup containing the pathfinding command and the parallel command.
   */
  public Command pathfindToWithHook(Location location, Command parallelCommand) {
    return pathfindToWithHook(location.getPose(), DEFAULT_CONSTRAINTS, parallelCommand);
  }

  /**
   * Generates a command to pathfind to a pose while running another command in parallel.
   * The parallel command will run until the pathfinding is complete.
   *
   * @param targetPose The target pose.
   * @param constraints Path constraints.
   * @param parallelCommand The command to run while moving.
   * @return A ParallelDeadlineGroup containing the pathfinding command and the parallel command.
   */
  public Command pathfindToWithHook(
      Pose2d targetPose, PathConstraints constraints, Command parallelCommand) {
    return Commands.deadline(pathfindTo(targetPose, constraints), parallelCommand);
  }

  /**
   * Adds a custom location of interest at runtime.
   *
   * @param name Name of the location.
   * @param pose Pose of the location.
   */
  public void addCustomLocation(String name, Pose2d pose) {
    customLocations.put(name, pose);
  }

  /**
   * Gets a custom location by name.
   *
   * @param name Name of the location.
   * @return The pose, or null if not found.
   */
  public Pose2d getCustomLocation(String name) {
    return customLocations.get(name);
  }
}
