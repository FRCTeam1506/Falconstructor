package frc.robot;

import frc.robot.commands.Drivetrain.Align;
import frc.robot.commands.Drivetrain.ArcadeDrive;
import frc.robot.commands.HorizIndexer.HorizIndex;
import frc.robot.commands.HorizIndexer.StopHorizIndexer;
import frc.robot.commands.Intake.ExtendAndIntake;
import frc.robot.commands.Intake.IntakeDefault;
import frc.robot.commands.Macros.IndexAndShoot;
import frc.robot.commands.Shifter.DefaultSetToHighGear;
import frc.robot.commands.Shifter.SetToHighGear;
import frc.robot.commands.Shifter.SetToLowGear;
import frc.robot.commands.Shooter.Shoot;
import frc.robot.commands.Shooter.StopShooter;
import frc.robot.commands.VertIndexer.StopVertIndexer;
import frc.robot.commands.VertIndexer.VertIndex;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.HorizIndexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shifter;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.VertIndexer;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  private final Joystick driver = new Joystick(0);
  private final Joystick operator = new Joystick(1);

  // The robot's subsystems and commands are defined here...
  protected static final Drivetrain drivetrain = new Drivetrain();
  protected static final Shifter shifter = new Shifter();
  protected static final Shooter shooter = new Shooter();
  protected static final Intake intake = new Intake();
  protected static final HorizIndexer horizIndexer = new HorizIndexer();
  protected static final VertIndexer vertIndexer = new VertIndexer();

  private final SendableChooser<Constants.Auto.Position> positionChooser = new SendableChooser<>();
  private final SendableChooser<Constants.Auto.Goal> goalChooser = new SendableChooser<>();

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
    new JoystickButton(driver, Constants.Playstation.TriangleButton.getID()).whileHeld(new Shoot(shooter, 36000.0));
    new JoystickButton(driver, Constants.Playstation.XButton.getID()).whileHeld(new HorizIndex(horizIndexer));
    new JoystickButton(driver, Constants.Playstation.CircleButton.getID()).whileHeld(new VertIndex(vertIndexer));
    new JoystickButton(driver, Constants.Playstation.LeftBumper.getID()).whileHeld(new ExtendAndIntake(intake));
    new JoystickButton(driver, Constants.Playstation.RightBumper.getID()).whileHeld(new SetToLowGear(shifter));
    new JoystickButton(driver, Constants.Playstation.BigButton.getID()).whileHeld(new IndexAndShoot(horizIndexer, vertIndexer, shooter));
    new JoystickButton(driver, Constants.Playstation.SquareButton.getID()).whenPressed(new Align(drivetrain).withTimeout(3.0));
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
      new DefaultSetToHighGear(shifter)
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

  private void setupAutonChooser() {}


  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return null;
  }
}
