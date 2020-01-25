/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.io.IOException;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import frc.robot.Constants.Playstation;
import frc.robot.commands.Drivetrain.LimitedArcadeDrive;
import frc.robot.commands.Auto.TestDrive;
import frc.robot.subsystems.*;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  public static final Drivetrain drivetrain = new Drivetrain();
  public static final Limelight limelight = new Limelight();

  private SendableChooser<Command> autoChooser = new SendableChooser<>();

  private final Command a_testDrive = new TestDrive(drivetrain);

  private Joystick driver = new Joystick(0);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   * @throws IOException
   */

  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
    // Set default commands
    setDefaultCommands();
    // Set Auton Configuration
    setAutoConfig();
  }

  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // [Auto] Aim and Shoot
    // Reasoning -- X marks the spot
    new JoystickButton(driver, Constants.Playstation.XButton.getID()).whenHeld(null);
    // [Auto] Ball Pickup
    // Reasoning -- Circle same shape as ball
    new JoystickButton(driver, Constants.Playstation.CircleButton.getID()).whenHeld(null);
  }

  private void setDefaultCommands() {
    drivetrain.setDefaultCommand(
      new LimitedArcadeDrive (
        drivetrain,
        () -> driver.getRawAxis(Playstation.LeftYAxis.getID()),
        () -> driver.getRawAxis(Playstation.RightXAxis.getID())
      )
    );
  }

  private void setAutoConfig() {
    // autoChooser.setDefaultOption("DRIVE fwd - TURN left", new FwdThenLeft(drivetrain));
    autoChooser.setDefaultOption("Test Drive", a_testDrive);
    autoChooser.addOption("Safe", null);
    autoChooser.addOption("Ambitious", null);
    autoChooser.addOption("Aim and Shoot", null);
    Shuffleboard.getTab("Autonomous").add(autoChooser);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
