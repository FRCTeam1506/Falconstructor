package frc.robot;

import frc.robot.Constants.Auto.Goal;
import frc.robot.Constants.Auto.Position;
import frc.robot.commands.Auton.Nothing;
import frc.robot.commands.Drivetrain.ArcadeDrive;
import frc.robot.commands.Drivetrain.DriveBackward;
import frc.robot.commands.Drivetrain.DriveForward;
import frc.robot.commands.Drivetrain.DriveStraight;
import frc.robot.commands.Drivetrain.FastTurnToAngleProfiled;
import frc.robot.commands.Drivetrain.TankDrive;
import frc.robot.commands.Drivetrain.TurnToAngle;
import frc.robot.commands.Drivetrain.TurnToAngleProfiled;
import frc.robot.commands.Macros.TurnToAngle2;
import frc.robot.commands.Shifter.DefaultSetToHighGear;
import frc.robot.commands.Shifter.DefaultSetToLowGear;
import frc.robot.commands.Shifter.SetToHighGear;
import frc.robot.commands.Shifter.SetToLowGear;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Shifter;
import frc.robot.utils.TrajectoryLoader;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;

/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  public static final Joystick driver = new Joystick(0);
  private final Joystick operator = new Joystick(1);
  private final Joystick testinator = new Joystick(2);

  // The robot's subsystems and commands are defined here...
  protected static final Drivetrain drivetrain = new Drivetrain();
  protected static final Shifter shifter = new Shifter();

  private SendableChooser<Constants.Auto.Position> positionChooser = new SendableChooser<>();
  private SendableChooser<Constants.Auto.Goal> goalChooser = new SendableChooser<>();

  private static final Command d_driveStraight = new DriveStraight(drivetrain);

  private Trajectory Six_Ball_1, Six_Ball_2;

  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    //? Configure the button bindings
    configureButtonBindings();

    //? Set Default Commands
    setDefaultCommands();

    //? Setup Auton Chooser
    setupAutonChooser();

    //? Create Trajectories
    createTrajectories();
  }

  private void configureButtonBindings() {
    //? Driver Controls
    new JoystickButton(driver, Constants.Playstation.LeftBumper.getID()).whileHeld(new SetToLowGear(shifter));
    new POVButton(driver, Constants.Playstation.EastPOVButton.getID()).whenPressed(new TurnToAngle(drivetrain, 90).withTimeout(5.0));
    new POVButton(driver, Constants.Playstation.WestPOVButton.getID()).whenPressed(new TurnToAngle(drivetrain, -90).withTimeout(5.0));
    new POVButton(driver, Constants.Playstation.SouthPOVButton.getID()).whenPressed(new TurnToAngle(drivetrain, 180).withTimeout(5.0));
    new JoystickButton(driver, Constants.Playstation.SquareButton.getID()).whileHeld(new TurnToAngle(drivetrain, -5)); // turn robot to left
    new JoystickButton(driver, Constants.Playstation.CircleButton.getID()).whileHeld(new TurnToAngle(drivetrain, 5)); // turn robot to right
  }


  private void setDefaultCommands() {

    drivetrain.setDefaultCommand(
      new ArcadeDrive(
        drivetrain,
        () -> -driver.getRawAxis(Constants.Playstation.LeftYAxis.getID()),
        () -> driver.getRawAxis(Constants.Playstation.RightXAxis.getID())
      )
    );

    // drivetrain.setDefaultCommand(
    //   new PIDCommand(
    //     new PIDController(
    //       Constants.Drivetrain.STABILIZATION_PID[0],
    //       Constants.Drivetrain.STABILIZATION_PID[1],
    //       Constants.Drivetrain.STABILIZATION_PID[2]
    //     ),
    //     drivetrain::getTurnRate,
    //     0.0,
    //     output -> drivetrain.arcadeDrive(-driver.getRawAxis(Constants.Playstation.LeftYAxis.getID()), output + driver.getRawAxis(Constants.Playstation.RightXAxis.getID())),
    //     drivetrain
    //   )
    // );

    shifter.setDefaultCommand(
      new DefaultSetToHighGear(shifter)
    );

  }

  private void setupAutonChooser() {

    //? Position Auton Chooser
    positionChooser.setDefaultOption("Nothing", Position.Nothing);
    positionChooser.addOption("Left", Position.Left);
    positionChooser.addOption("Middle", Position.Middle);
    positionChooser.addOption("Right", Position.Right);
    Shuffleboard.getTab("Autonomous").add("Position", positionChooser);

    //? Goal Auton Chooser
    goalChooser.setDefaultOption("Safe", Goal.Safe);
    goalChooser.addOption("Ambitious", Goal.Ambitious);
    Shuffleboard.getTab("Autonomous").add("Goal", goalChooser);

  }

  private void createTrajectories() {
    this.Six_Ball_1 = TrajectoryLoader.loadTrajectoryFromFile("1_u-turn");
    this.Six_Ball_2 = TrajectoryLoader.loadTrajectoryFromFile("2_to-front-trench");
  }

  public Command getAutonomousCommand() {
    //? Reset Sensors
    drivetrain.resetAll();
    return new ParallelCommandGroup(
      new DefaultSetToHighGear(shifter),
      // test_fwd()
      // standardRamseteCommand("fwd")
      standardRamseteRevCommand("fwd"),
      new RamseteCommand(
        TrajectoryLoader.loadTrajectoryFromFile("u_curve_rev"),
        drivetrain::getPose,
        new RamseteController(),
        new SimpleMotorFeedforward(
          Constants.Drivetrain.kS,
          Constants.Drivetrain.kV,
          Constants.Drivetrain.kA
        ),
        Constants.Drivetrain.kDriveKinematics,
        drivetrain::getWheelSpeedsRev,
        new PIDController(1.1, 0.01, 0.15),
        new PIDController(1.5, 0.01, 0.05),
        drivetrain::tankDriveVoltsRev,
        drivetrain
      )
    );
  }

  private Command test_fwd() {
    return new RamseteCommand(
      TrajectoryLoader.loadTrajectoryFromFile("Unnamed"),
      drivetrain::getPose,
      new RamseteController(2.0, 0.7), // 2.3
      new SimpleMotorFeedforward(
        Constants.Drivetrain.kS,
        Constants.Drivetrain.kV,
        Constants.Drivetrain.kA
      ),
      Constants.Drivetrain.kDriveKinematics,
      drivetrain::getWheelSpeeds,
      new PIDController(1.5, 0.01, 0.05),
      new PIDController(1.1, 0.01, 0.15),
      drivetrain::tankDriveVolts,
      drivetrain
    ).andThen(() -> drivetrain.arcadeDrive(0.0, 0.0));
  }

  private Command test_rev() {
    return new RamseteCommand(
      TrajectoryLoader.loadTrajectoryFromFile("Unnamed"),
      drivetrain::getPose,
      new RamseteController(2.0, 0.7), // 2.3
      new SimpleMotorFeedforward(
        Constants.Drivetrain.kS,
        Constants.Drivetrain.kV,
        Constants.Drivetrain.kA
      ),
      Constants.Drivetrain.kDriveKinematics,
      drivetrain::getWheelSpeedsRev,
      new PIDController(1.1, 0.01, 0.15),
      new PIDController(1.5, 0.01, 0.05),
      drivetrain::tankDriveVoltsRev,
      drivetrain
    ).andThen(() -> drivetrain.arcadeDrive(0.0, 0.0));
  }

  private RamseteCommand standardRamseteCommand(Trajectory trajectory) {
    return new RamseteCommand(
      trajectory,
      drivetrain::getPose,
      new RamseteController(2.0, 0.7), // 2.3
      new SimpleMotorFeedforward(
        Constants.Drivetrain.kS,
        Constants.Drivetrain.kV,
        Constants.Drivetrain.kA
      ),
      Constants.Drivetrain.kDriveKinematics,
      drivetrain::getWheelSpeeds,
      new PIDController(1.5, 0.01, 0.15),
      new PIDController(1.55, 0.047, 0.15),
      drivetrain::tankDriveVolts,
      drivetrain
    );
  }

  private RamseteCommand fastRamseteCommand(Trajectory trajectory) {
    return new RamseteCommand(
      trajectory,
      drivetrain::getPose,
      new RamseteController(2.0, 0.7), // 2.3
      new SimpleMotorFeedforward(
        Constants.Drivetrain.kS,
        Constants.Drivetrain.kV,
        Constants.Drivetrain.kA
      ),
      Constants.Drivetrain.kDriveKinematics,
      drivetrain::getWheelSpeeds,
      new PIDController(3.0, 0.01, 0.15),
      new PIDController(3.0, 0.047, 0.15),
      drivetrain::fastTankDriveVolts,
      drivetrain
    );
  } 

  private RamseteCommand standardRamseteCommand(String name) {
    return new RamseteCommand(
      TrajectoryLoader.loadTrajectoryFromFile(name),
      drivetrain::getPose,
      new RamseteController(2.0, 0.7), // 2.3
      new SimpleMotorFeedforward(
        Constants.Drivetrain.kS,
        Constants.Drivetrain.kV,
        Constants.Drivetrain.kA
      ),
      Constants.Drivetrain.kDriveKinematics,
      drivetrain::getWheelSpeeds,
      new PIDController(1.5, 0.01, 0.15),
      new PIDController(1.55, 0.047, 0.15),
      drivetrain::tankDriveVolts,
      drivetrain
    );
  } 

  private Command standardRamseteRevCommand(String name) {
    return new RamseteCommand(
      TrajectoryLoader.createReverseTrajectory(name),
      drivetrain::getPose,
      new RamseteController(2.0, 0.7), // 2.3
      new SimpleMotorFeedforward(
        Constants.Drivetrain.kS,
        Constants.Drivetrain.kV,
        Constants.Drivetrain.kA
      ),
      Constants.Drivetrain.kDriveKinematics,
      drivetrain::getWheelSpeeds,
      new PIDController(1.5, 0.01, 0.15),
      new PIDController(1.55, 0.047, 0.15),
      drivetrain::tankDriveVolts,
      drivetrain
    ).andThen(() -> drivetrain.arcadeDrive(0.0, 0.0));
  }

  public static Command getDriveStraight() {
    return d_driveStraight;
  }
}
