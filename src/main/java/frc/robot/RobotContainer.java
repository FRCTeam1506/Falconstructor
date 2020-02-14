/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.io.IOException;
import java.nio.file.Path;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;

import frc.robot.commands.Intake.IntakeIntake;
import frc.robot.commands.Limelight.Align;
import frc.robot.commands.Limelight.AlignFinite;
import frc.robot.commands.Limelight.CorrectDistance;
import frc.robot.commands.Limelight.GyroAlign;
import frc.robot.commands.Limelight.Idle;
import frc.robot.commands.Limelight.StandardAlignAndSeek;
import frc.robot.commands.Macros.AimAndShoot;
import frc.robot.commands.Macros.Arrange;
import frc.robot.commands.Macros.GyroAlignAndCorrectDist;
import frc.robot.commands.Macros.IntakeAndShoot;
import frc.robot.commands.Shooter.FullSend;
import frc.robot.commands.Shooter.Shoot;
import frc.robot.commands.Auto.Nothing;
import frc.robot.commands.Drivetrain.DriveToDist;
import frc.robot.commands.Drivetrain.DriveToDistProfiled;
import frc.robot.commands.Drivetrain.FastArcadeDrive;
import frc.robot.commands.Drivetrain.LimitedArcadeDrive;
import frc.robot.commands.Drivetrain.TurnToAngle;
import frc.robot.commands.Drivetrain.TurnToAngleProfiled;
import frc.robot.commands.Indexer.Index;

import frc.robot.subsystems.*;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  public static boolean aligned;
  // The robot's subsystems and commands are defined here...
  public static final Drivetrain drivetrain = new Drivetrain();
  public static final Limelight limelight = new Limelight();
  public static final Shooter shooter = new Shooter();
  public static final Intake intake = new Intake();
  public static final Indexer indexer = new Indexer();

  private SendableChooser<Command> autoChooser = new SendableChooser<>();

  private final Command l_correctDistance = new CorrectDistance(drivetrain, limelight);
  private final Command l_align = new Align(drivetrain, limelight);
  private final Command l_alignFinite = new AlignFinite(drivetrain, limelight);
  private final Command l_standardAlignAndSeek = new StandardAlignAndSeek(drivetrain, limelight);
  private final Command l_idle = new Idle(drivetrain, limelight);
  public static final Command l_gyroAlign = new GyroAlign(drivetrain, limelight);

  private final Command s_fullSend = new FullSend(shooter);

  private final Command a_nothing = new Nothing();

  private final Command m_intakeAndShoot = new IntakeAndShoot(intake, indexer, shooter);
  private final Command m_aimAndShoot = new AimAndShoot(drivetrain, limelight, indexer, intake, shooter);
  private final Command m_arrange = new Arrange(drivetrain, limelight, 1234.0);
  private final Command m_gyroAlignAndCorrectDist = new GyroAlignAndCorrectDist(drivetrain, limelight);

  public static Joystick driver = new Joystick(0);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   * @throws IOException
   */

  public RobotContainer() {
    // Create list of paths
    createPathsList();
    // Configure the button bindings
    configureButtonBindings();
    // Set default commands
    setDefaultCommands();
    // Set Auton Configuration
    setAutoConfig();

    smartdashboard();
  }

  public static void setAlign(boolean value) {
    aligned = value;
  }

  public static boolean getAlign() {
    return aligned;
  }
 
  private void createPathsList() {
    // paths.put("A", new RamseteCommand(, pose, controller, feedforward, kinematics, wheelSpeeds, leftController, rightController, outputVolts, requirements));
    // paths.put("B", new RamseteCommand(trajectory, pose, controller, feedforward, kinematics, wheelSpeeds, leftController, rightController, outputVolts, requirements));
  }

  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // new JoystickButton(driver, Constants.Playstation.LeftBumper.getID()).whileHeld(
    //   new FastArcadeDrive(
    //     drivetrain,
    //     () -> driver.getRawAxis(Constants.Playstation.LeftYAxis.getID()),
    //     () -> driver.getRawAxis(Constants.Playstation.RightXAxis.getID())
    //   )
    // );

    // [Auto] Aim
    // Reasoning -- X marks the spot
    new JoystickButton(driver, Constants.Playstation.XButton.getID())
      .whenPressed(l_align)
      .whenReleased(l_idle);
    // [Auto] Ball Pickup
    // Reasoning -- Circle same shape as ball
    new JoystickButton(driver, Constants.Playstation.CircleButton.getID()).whileHeld(m_arrange);
    // [Shooter] Shoot Ball
    new JoystickButton(driver, Constants.Playstation.RightBumper.getID()).whileHeld(s_fullSend);
    // [Macro] Intake and Shoot
    new JoystickButton(driver, Constants.Playstation.TriangleButton.getID()).whenHeld(m_intakeAndShoot);
    // [Macro] Aim and Shoot
    new JoystickButton(driver, Constants.Playstation.SquareButton.getID()).whenHeld(m_aimAndShoot);

    new JoystickButton(driver, Constants.Playstation.BigButton.getID()).whileHeld(m_gyroAlignAndCorrectDist);
    new JoystickButton(driver, Constants.Playstation.LeftButton.getID()).whenPressed(
      new TurnToAngle(
        drivetrain,
        limelight
      ).withTimeout(5)
    );

    new JoystickButton(driver, Constants.Playstation.RightButton.getID()).whenPressed(
      new TurnToAngleProfiled(
        drivetrain,
        drivetrain.getHeading() + -limelight.getX()
        ).withTimeout(5)
    );

    // new POVButton(driver, Constants.Playstation.NorthPOVButton.getID()).whenPressed(
    //   new DriveToDist(
    //     drivetrain,
    //     targetDistMeters
    //   ).withTimeout(5)
    // );

    // new POVButton(driver, Constants.Playstation.SouthPOVButton.getID()).whenPressed(
    //   new DriveToDistProfiled(
    //     drivetrain,
    //     targetDistMeters
    //   ).withTimeout(5)
    // );

  }

  private void setDefaultCommands() {
    drivetrain.setDefaultCommand(
      new LimitedArcadeDrive(
        drivetrain,
        () -> driver.getRawAxis(Constants.Playstation.LeftYAxis.getID()),
        () -> driver.getRawAxis(Constants.Playstation.RightXAxis.getID())
      )
    );

    shooter.setDefaultCommand(
      new Shoot(
        shooter,
        // () -> driver.getRawAxis(Playstation.RightYAxis.getID())
        () -> 0.0
      )
    );

    intake.setDefaultCommand(
      new IntakeIntake(
        intake,
        () -> (driver.getRawAxis(Constants.Playstation.RightTrigger.getID()) + 1.0) * -1.0
      )
    );
  }

  private void setAutoConfig() {
    // RamseteCommand ramseteCommand = new RamseteCommand(trajectory, pose, controller, feedforward, kinematics, wheelSpeeds, leftController, rightController, outputVolts, requirements);
	// autoChooser.setDefaultOption("DRIVE fwd - TURN left", new FwdThenLeft(drivetrain));
	  autoChooser.setDefaultOption("1auto", null);
    autoChooser.addOption("Safe", null);
    autoChooser.addOption("Aim and Shoot", null);
    Shuffleboard.getTab("Autonomous").add(autoChooser);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  // public Command getAutonomousCommand() {
  //   return autoChooser.getSelected();
  // }

  public Command getAutonomousCommand() {
    String trajectoryJSON = "output/";
    Trajectory trajectory = null;
    String choice = autoChooser.getSelected().toString();

    switch (choice) {
      case "1auto":
        trajectoryJSON += choice+".wpilib.json";
        try {
          	Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
            trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
        } catch (IOException ex) {
          	DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON, ex.getStackTrace());
        }
        break;
    
      default:
        trajectoryJSON += choice+".wpilib.json";
        try {
          	Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
          	trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
        } catch (IOException ex) {
          	DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON, ex.getStackTrace());
        }
        break;
    }

    RamseteCommand ramseteCommand = new RamseteCommand(
        trajectory,
        drivetrain::getPose, 
        new RamseteController(2.0, 0.7), 
        new SimpleMotorFeedforward(
            Constants.Drivetrain.kS,
            Constants.Drivetrain.kV,
            Constants.Drivetrain.kA
        ), 
        Constants.Drivetrain.kDriveKinematics, 
        drivetrain::getWheelSpeeds, 
        new PIDController(3.0, 0.0, 0.0), 
        new PIDController(3.0, 0.0, 0.0), 
        drivetrain::tankDrive, 
        drivetrain
    );
    return ramseteCommand.andThen(() -> drivetrain.tankDrive(0,0));
  }

  public void smartdashboard() {
    System.out.println("Heading " + drivetrain.getHeading());
    System.out.println("Limelight-X " + -limelight.getX());
    SmartDashboard.putNumber("[+]-Target-Angle-Degrees", drivetrain.getHeading() + -limelight.getX());
  }

  public static Command getGyroAlign() {
    return l_gyroAlign;
  }

}
