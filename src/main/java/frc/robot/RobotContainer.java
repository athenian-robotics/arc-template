// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathfindingCommand;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.constants.OperationConfig;
import frc.robot.constants.PathGenerationConstants.Location;
import frc.robot.constants.RobotConstants.RuntimeConstants;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOLimelight;
import frc.robot.tunerconstants.TunerConstantsHelper;
import frc.robot.util.PathGeneration;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // -- Subsystems --
  private final Drive drive;
  private final Vision vision;
  private final PathGeneration pathGeneration;

  // -- Controllers --
  private final CommandJoystick driveJoystick =
      new CommandJoystick(OperationConfig.LEFT_JOYSTICK_PORT);
  private final CommandJoystick steerJoystick =
      new CommandJoystick(OperationConfig.MIDDLE_JOYSTICK_PORT);
  private final CommandJoystick operatorJoystick =
      new CommandJoystick(OperationConfig.RIGHT_JOYSTICK_PORT);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (RuntimeConstants.currentMode) {
      case REAL: // Real robot, instantiate hardware IO implementations
        vision = new Vision(new VisionIOLimelight());
        drive = new Drive(
            vision,
            new GyroIOPigeon2(),
            new ModuleIOTalonFX(TunerConstantsHelper.FrontLeft),
            new ModuleIOTalonFX(TunerConstantsHelper.FrontRight),
            new ModuleIOTalonFX(TunerConstantsHelper.BackLeft),
            new ModuleIOTalonFX(TunerConstantsHelper.BackRight));
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        vision = new Vision(new VisionIO() {});
        drive = new Drive(
            vision,
            new GyroIO() {},
            new ModuleIOSim(TunerConstantsHelper.FrontLeft),
            new ModuleIOSim(TunerConstantsHelper.FrontRight),
            new ModuleIOSim(TunerConstantsHelper.BackLeft),
            new ModuleIOSim(TunerConstantsHelper.BackRight));
        break;

      default:
        // Replayed robot, disable IO implementations
        vision = new Vision(new VisionIO() {});
        drive = new Drive(
            vision,
            new GyroIO() {},
            new ModuleIO() {},
            new ModuleIO() {},
            new ModuleIO() {},
            new ModuleIO() {});
    }

    pathGeneration = new PathGeneration();

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Set up SysId routines
    addSysIds();

    // Configure the button bindings
    configureJoystickBindings();

    PathfindingCommand.warmupCommand().schedule();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureJoystickBindings() {
    // Default command, normal field-relative drive
    drive.setDefaultCommand(
        OperationConfig.IS_HEADING_BASED_DRIVE
        ?
            DriveCommands.joystickDriveAtAngle(
                drive,
                () -> driveJoystick.getY(),
                () -> driveJoystick.getX(),
                () -> new Rotation2d(-steerJoystick.getY(), -steerJoystick.getX()),
                () -> steerJoystick.getMagnitude() >= OperationConfig.HEADING_DEADBAND
            )
        :
            DriveCommands.joystickDrive(
                drive,
                () -> -driveJoystick.getY(),
                () -> -driveJoystick.getX(),
                () -> -steerJoystick.getX()
            )
        );

    // Reset gyro to 0°
    steerJoystick
        .button(OperationConfig.UNBOUND)
        .onTrue(
            Commands.runOnce(
                () -> drive.setPose(new Pose2d(drive.getPose().getTranslation(), new Rotation2d())), 
                drive)
            .ignoringDisable(true)
        );

    /*-----The following code is untested-----*/
    // Should return the bot to its initial position
    driveJoystick
        .button(OperationConfig.UNBOUND)
        .whileTrue(
            Commands.runOnce(() -> System.out.println("Running path gen"))
                .andThen(pathGeneration.pathfindTo(Location.TEST_POSE)));

    // Might work better
    driveJoystick
        .button(OperationConfig.UNBOUND)
        .onTrue(pathGeneration.pathfindToSimple(drive::getPose, Location.TEST_POSE, 0.0));
  }

  /** A method that runs when the robot is disabled. It turns off all subsystems. */
  public void disable() {

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }

  private void addSysIds() {
    autoChooser.addOption(
        "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    autoChooser.addOption(
        "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        drive.sysIdQuasistaticDrive(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        drive.sysIdQuasistaticDrive(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Drive SysId (Dynamic Forward)", drive.sysIdDynamicDrive(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)", drive.sysIdDynamicDrive(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Turn SysId (Quasistatic Forward)",
        drive.sysIdQuasistaticRotate(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Turn SysId (Quasistatic Reverse)",
        drive.sysIdQuasistaticRotate(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Turn SysId (Dynamic Forward)", drive.sysIdDynamicRotate(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Turn SysId (Dynamic Reverse)", drive.sysIdDynamicRotate(SysIdRoutine.Direction.kReverse));
  }
}
