package frc.robot;

import frc.robot.Constants.Auto.Position;
import frc.robot.commands.Auton.LeftAuton;
import frc.robot.commands.Auton.MiddleAuton;
import frc.robot.commands.Auton.Nothing;
import frc.robot.commands.Auton.RightAuton;
import frc.robot.commands.Drivetrain.Align;
import frc.robot.commands.Drivetrain.ArcadeDrive;
import frc.robot.commands.HorizIndexer.HorizIndex;
import frc.robot.commands.HorizIndexer.HorizIndexLeft;
import frc.robot.commands.HorizIndexer.HorizIndexRev;
import frc.robot.commands.HorizIndexer.HorizIndexRevCycle;
import frc.robot.commands.HorizIndexer.StopHorizIndexer;
import frc.robot.commands.Intake.ExtendAndIntake;
import frc.robot.commands.Intake.ExtendAndOutake;
import frc.robot.commands.Intake.IntakeDefault;
import frc.robot.commands.Macros.IndexAndShoot;
import frc.robot.commands.Macros.TestMaster;
import frc.robot.commands.Macros.Unjam;
import frc.robot.commands.Shifter.DefaultSetToHighGear;
import frc.robot.commands.Shifter.DefaultSetToLowGear;
import frc.robot.commands.Shifter.SetToHighGear;
import frc.robot.commands.Shifter.SetToLowGear;
import frc.robot.commands.Shooter.Shoot;
import frc.robot.commands.Shooter.StopShooter;
import frc.robot.commands.VertIndexer.StopVertIndexer;
import frc.robot.commands.VertIndexer.VertIndex;
import frc.robot.commands.VertIndexer.VertIndexRev;

import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.HorizIndexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shifter;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.VertIndexer;
import frc.robot.utils.TrajectoryLoader;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
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

  // The robot's subsystems and commands are defined here...
  protected static final Drivetrain drivetrain = new Drivetrain();
  protected static final Shifter shifter = new Shifter();
  protected static final Shooter shooter = new Shooter();
  protected static final Intake intake = new Intake();
  protected static final HorizIndexer horizIndexer = new HorizIndexer();
  protected static final VertIndexer vertIndexer = new VertIndexer();

  private SendableChooser<Constants.Auto.Position> positionChooser = new SendableChooser<>();
  private SendableChooser<Constants.Auto.Goal> goalChooser = new SendableChooser<>();

  protected final Command test = new TestMaster(drivetrain, shifter, intake, horizIndexer, vertIndexer, shooter);
  private final Command m_extendAndIntake = new ExtendAndIntake(intake);
  private final Command m_extendAndOutake = new ExtendAndOutake(intake);

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
  }

  private void configureButtonBindings() {
    new JoystickButton(driver, Constants.Playstation.TriangleButton.getID()).whileHeld(new Shoot(shooter, 22000.0));
    new JoystickButton(driver, Constants.Playstation.XButton.getID()).whileHeld(new HorizIndex(horizIndexer));
    new JoystickButton(driver, Constants.Playstation.CircleButton.getID()).whileHeld(new VertIndex(vertIndexer));
    // new JoystickButton(operator, Constants.Playstation.LeftBumper.getID()).whileHeld(new ExtendAndIntake(intake));
    new JoystickButton(operator, Constants.Playstation.LeftBumper.getID()).whileHeld(m_extendAndIntake);
    new JoystickButton(operator, Constants.Playstation.LeftBumper.getID())
      .whileActiveContinuous(m_extendAndOutake, true)
      .and(new POVButton(operator, Constants.Playstation.NorthPOVButton.getID()))
      .cancelWhenActive(m_extendAndIntake);
    new JoystickButton(driver, Constants.Playstation.RightBumper.getID()).whileHeld(new SetToHighGear(shifter));
    new JoystickButton(operator, Constants.Playstation.RightBumper.getID()).whileHeld(new IndexAndShoot(intake, horizIndexer, vertIndexer, shooter));
    new JoystickButton(driver, Constants.Playstation.SquareButton.getID()).whenPressed(new Align(drivetrain).withTimeout(3.0));
    // new JoystickButton(operator, Constants.Playstation.XButton.getID()).whileHeld(new VertIndexRev(vertIndexer));
    // new JoystickButton(operator, Constants.Playstation.CircleButton.getID()).whileHeld(new HorizIndexRev(horizIndexer));
    new JoystickButton(operator, Constants.Playstation.XButton.getID()).whileHeld(new Unjam(horizIndexer, vertIndexer, intake));
    new JoystickButton(operator, Constants.Playstation.CircleButton.getID()).whileHeld(new ExtendAndOutake(intake));
    new JoystickButton(operator, Constants.Playstation.TriangleButton.getID()).whileHeld(new HorizIndexRevCycle(horizIndexer));
  }


  private void setDefaultCommands() {

    drivetrain.setDefaultCommand(
      new ArcadeDrive(
        drivetrain,
        () -> -driver.getRawAxis(Constants.Playstation.LeftYAxis.getID()),
        () -> driver.getRawAxis(Constants.Playstation.RightXAxis.getID())
      )
    );

    shifter.setDefaultCommand(
      new DefaultSetToLowGear(shifter)
    );

    shooter.setDefaultCommand(
      new StopShooter(shooter)
    );

    intake.setDefaultCommand(
      new IntakeDefault(intake)
    );

    horizIndexer.setDefaultCommand(
      new StopHorizIndexer(horizIndexer)
    );

    vertIndexer.setDefaultCommand(
      new StopVertIndexer(vertIndexer)
    );

  }

  private void setupAutonChooser() {
    positionChooser.setDefaultOption("Nothing", Position.Nothing);
    positionChooser.addOption("Left", Position.Left);
    positionChooser.addOption("Middle", Position.Middle);
    positionChooser.addOption("Right", Position.Right);
    Shuffleboard.getTab("Autonomous").add("Position", positionChooser);
  }


  public Command getAutonomousCommand() {
    Trajectory trajectory = TrajectoryLoader.loadTrajectoryFromFile("Unnamed");
    RamseteCommand ramseteCommand = new RamseteCommand(
      trajectory,
      drivetrain::getPose,
      new RamseteController(1.7, 0.7),
      new SimpleMotorFeedforward(
        Constants.Drivetrain.kS,
        Constants.Drivetrain.kV,
        Constants.Drivetrain.kA
      ),
      Constants.Drivetrain.kDriveKinematics,
      drivetrain::getWheelSpeeds,
      new PIDController(3.0, 0.0, 0.0),
      new PIDController(3.0, 0.0, 0.0),
      drivetrain::tankDriveVolts,
      drivetrain
    );
    Position pos = positionChooser.getSelected();
    // if(pos == Position.Nothing) return ramseteCommand.andThen(() -> drivetrain.tankDrive(0.0, 0.0));
    // else if(pos == Position.Left) return new LeftAuton(drivetrain, intake, horizIndexer, vertIndexer, shooter);
    // else if(pos == Position.Middle) return new MiddleAuton(drivetrain, intake, horizIndexer, vertIndexer, shooter);
    // else if(pos == Position.Right) return new RightAuton(drivetrain, intake, horizIndexer, vertIndexer, shooter);
    // else return new Nothing();
    return ramseteCommand.andThen(() -> drivetrain.tankDrive(0.0, 0.0));
  }
}
