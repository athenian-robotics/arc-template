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

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.RuntimeConstants;
import frc.robot.subsystems.Drive;
import org.littletonrobotics.junction.Logger;
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

  // -- Controllers --
  private final CommandJoystick driveJoystick =
      new CommandJoystick(ControllerConstants.JOYSTICK_LEFT_PORT);
  private final CommandJoystick steerJoystick =
      new CommandJoystick(ControllerConstants.JOYSTICK_MIDDLE_PORT);
  private final CommandJoystick operatorJoystick =
      new CommandJoystick(ControllerConstants.JOYSTICK_RIGHT_PORT);

  // Dashboard inputs
  // private final LoggedDashboardChooser<Command> autoChooser =
  // new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

  private final LoggedDashboardChooser<Command> testChooser = new LoggedDashboardChooser<>("Tests");

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    drive = new Drive();

    testChooser.addOption(
        "Test drive",
        Commands.run(
            () -> {
              drive.test();
            },
            drive));

    // switch (RuntimeConstants.currentMode) {
    //   case REAL:
    //     // Real robot, instantiate hardware IO implementations
    //     drive =
    //         new Drive(
    //             new GyroIOPigeon2(),
    //             new ModuleIOTalonFX(TunerConstants.FrontLeft),
    //             new ModuleIOTalonFX(TunerConstants.FrontRight),
    //             new ModuleIOTalonFX(TunerConstants.BackLeft),
    //             new ModuleIOTalonFX(TunerConstants.BackRight));
    //     break;

    //   case SIM:
    //     // Sim robot, instantiate physics sim IO implementations
    //     drive =
    //         new Drive(
    //             new GyroIO() {},
    //             new ModuleIOSim(TunerConstants.FrontLeft),
    //             new ModuleIOSim(TunerConstants.FrontRight),
    //             new ModuleIOSim(TunerConstants.BackLeft),
    //             new ModuleIOSim(TunerConstants.BackRight));
    //     break;

    //   default:
    //     // Replayed robot, disable IO implementations
    //     drive =
    //         new Drive(
    //             new GyroIO() {},
    //             new ModuleIO() {},
    //             new ModuleIO() {},
    //             new ModuleIO() {},
    //             new ModuleIO() {});
    //     break;

    // Set up auto routines
    // autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Set up SysId routines
    // autoChooser.addOption(
    //     "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    // autoChooser.addOption(
    //     "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    // autoChooser.addOption(
    //     "Drive SysId (Quasistatic Forward)",
    //     drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    // autoChooser.addOption(
    //     "Drive SysId (Quasistatic Reverse)",
    //     drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    // autoChooser.addOption(
    //     "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    // autoChooser.addOption(
    //     "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    // Configure the button bindings
    configureTeleopBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureTeleopBindings() {
    // JoystickCommands.driveWithTurnAngular(
    //     driveJoystick::getX, driveJoystick::getY, steerJoystick::getX, drive);
  }

  public void logMetadata() {
    Logger.recordMetadata("Event Name", DriverStation.getEventName());
    Logger.recordMetadata("Driver Station Location", DriverStation.getLocation() + "");
    Logger.recordMetadata("Match Number", DriverStation.getMatchNumber() + "");
    Logger.recordMetadata("Match Type", DriverStation.getMatchType() + "");
    Logger.recordMetadata("Replay Number", DriverStation.getReplayNumber() + "");
    Logger.recordMetadata("Robot Mode", "" + RuntimeConstants.CURRENT_MODE);
  }
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  // public Command getAutonomousCommand() {
  //   return autoChooser.get();
  // }
}
