package frc.robot;

import frc.robot.Constants.Auto.Goal;
import frc.robot.Constants.Auto.Position;
import frc.robot.commands.Auton.LeftAuton;
import frc.robot.commands.Auton.MiddleAuton;
import frc.robot.commands.Auton.Nothing;
import frc.robot.commands.Auton.RightAuton;
import frc.robot.commands.Climber.Control;
import frc.robot.commands.Climber.Default;
import frc.robot.commands.Climber.Extend;
import frc.robot.commands.Climber.Retract;
import frc.robot.commands.Climber.SetClimbingHeight;
import frc.robot.commands.Climber.SetInitialHeight;
import frc.robot.commands.Drivetrain.Align;
import frc.robot.commands.Drivetrain.ArcadeDrive;
import frc.robot.commands.Drivetrain.DriveBackward;
import frc.robot.commands.Drivetrain.DriveForward;
import frc.robot.commands.Drivetrain.DriveStraight;
import frc.robot.commands.Drivetrain.TurnToAngle;
import frc.robot.commands.Drivetrain.TurnToAngleProfiled;
import frc.robot.commands.HorizIndexer.HorizIndex;
import frc.robot.commands.HorizIndexer.HorizIndexLeft;
import frc.robot.commands.HorizIndexer.HorizIndexRev;
import frc.robot.commands.HorizIndexer.HorizIndexRevCycle;
import frc.robot.commands.HorizIndexer.StopHorizIndexer;
import frc.robot.commands.Intake.ExtendAndIntake;
import frc.robot.commands.Intake.ExtendAndOutake;
import frc.robot.commands.Intake.IntakeDefault;
import frc.robot.commands.Intake.IntakeIntake;
import frc.robot.commands.Macros.IndexAndShoot;
import frc.robot.commands.Macros.TestMaster;
import frc.robot.commands.Macros.TurnToAngle2;
import frc.robot.commands.Macros.Unjam;
import frc.robot.commands.Macros.Tests.TestMechanisms;
import frc.robot.commands.Shifter.DefaultSetToHighGear;
import frc.robot.commands.Shifter.DefaultSetToLowGear;
import frc.robot.commands.Shifter.SetToHighGear;
import frc.robot.commands.Shifter.SetToLowGear;
import frc.robot.commands.Shooter.Shoot;
import frc.robot.commands.Shooter.StopShooter;
import frc.robot.commands.VertIndexer.StopVertIndexer;
import frc.robot.commands.VertIndexer.VertIndex;
import frc.robot.commands.VertIndexer.VertIndexRev;
import frc.robot.subsystems.Climber;
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
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
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
  private final Joystick testinator = new Joystick(2);

  // The robot's subsystems and commands are defined here...
  protected static final Drivetrain drivetrain = new Drivetrain();
  protected static final Shifter shifter = new Shifter();
  protected static final Shooter shooter = new Shooter();
  protected static final Intake intake = new Intake();
  protected static final HorizIndexer horizIndexer = new HorizIndexer();
  protected static final VertIndexer vertIndexer = new VertIndexer();
  protected static final Climber climber = new Climber();

  private SendableChooser<Constants.Auto.Position> positionChooser = new SendableChooser<>();
  private SendableChooser<Constants.Auto.Goal> goalChooser = new SendableChooser<>();

  protected final Command test = new TestMaster(drivetrain, shifter, intake, horizIndexer, vertIndexer, shooter);
  private final Command m_extendAndIntake = new ExtendAndIntake(intake);
  private final Command m_extendAndOutake = new ExtendAndOutake(intake);
  private static final Command d_driveStraight = new DriveStraight(drivetrain);

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

    // setupDashboard();
  }

  private void configureButtonBindings() {
    //? Driver Controls
    // new JoystickButton(driver, Constants.Playstation.TriangleButton.getID()).whileHeld(new Shoot(shooter, 22000.0));
    new JoystickButton(driver, Constants.Playstation.TriangleButton.getID()).whenPressed(new Align(drivetrain).withTimeout(3.0));
    new JoystickButton(driver, Constants.Playstation.XButton.getID()).whileHeld(new HorizIndex(horizIndexer));
    // new JoystickButton(driver, Constants.Playstation.CircleButton.getID()).whileHeld(new VertIndex(vertIndexer));
    new JoystickButton(driver, Constants.Playstation.LeftBumper.getID()).whileHeld(new SetToLowGear(shifter));
    // new JoystickButton(driver, Constants.Playstation.LeftBumper.getID()).whenHeld(d_driveStraight);
    new JoystickButton(driver, Constants.Playstation.RightBumper.getID()).whileHeld(m_extendAndIntake);
    new POVButton(driver, Constants.Playstation.EastPOVButton.getID()).whenPressed(new TurnToAngle(drivetrain, 90).withTimeout(5.0));
    new POVButton(driver, Constants.Playstation.WestPOVButton.getID()).whenPressed(new TurnToAngle(drivetrain, -90).withTimeout(5.0));
    new POVButton(driver, Constants.Playstation.SouthPOVButton.getID()).whenPressed(new TurnToAngle(drivetrain, 180).withTimeout(5.0));
    new JoystickButton(driver, Constants.Playstation.SquareButton.getID()).whileHeld(new TurnToAngle(drivetrain, -5)); // turn robot to left
    new JoystickButton(driver, Constants.Playstation.CircleButton.getID()).whileHeld(new TurnToAngle(drivetrain, 5)); // turn robot to right

    //? Operator Controls
    // new JoystickButton(operator, Constants.Playstation.LeftBumper.getID()).whileHeld(new ExtendAndIntake(intake));
    new JoystickButton(operator, Constants.Playstation.LeftBumper.getID()).whileHeld(m_extendAndIntake);
    // new JoystickButton(operator, Constants.Playstation.LeftBumper.getID())
    //   .whileActiveContinuous(m_extendAndOutake, true)
    //   .and(new POVButton(operator, Constants.Playstation.NorthPOVButton.getID()))
    //   .cancelWhenActive(m_extendAndIntake);
    new JoystickButton(operator, Constants.Playstation.RightBumper.getID()).whileHeld(new IndexAndShoot(intake, horizIndexer, vertIndexer, shooter));
    // new JoystickButton(operator, Constants.Playstation.XButton.getID()).whileHeld(new VertIndexRev(vertIndexer));
    // new JoystickButton(operator, Constants.Playstation.CircleButton.getID()).whileHeld(new HorizIndexRev(horizIndexer));
    new JoystickButton(operator, Constants.Playstation.XButton.getID()).whileHeld(new Unjam(horizIndexer, vertIndexer, intake));
    new JoystickButton(operator, Constants.Playstation.CircleButton.getID()).whileHeld(new ExtendAndOutake(intake));
    new JoystickButton(operator, Constants.Playstation.TriangleButton.getID()).whileHeld(new HorizIndexRevCycle(horizIndexer));
    new JoystickButton(operator, Constants.Playstation.BigButton.getID()).whenPressed(new Extend(climber));
    new JoystickButton(operator, Constants.Playstation.MiddleButton.getID()).whenPressed(new Retract(climber));
    new POVButton(operator, Constants.Playstation.NorthPOVButton.getID()).whileHeld(new SetClimbingHeight(climber));
    new POVButton(operator, Constants.Playstation.SouthPOVButton.getID()).whileHeld(new SetInitialHeight(climber));

    //? Test Controller Controls
    new JoystickButton(testinator, Constants.Playstation.BigButton.getID()).whileHeld(new TestMechanisms(intake, horizIndexer, vertIndexer, shooter));
    // new POVButton(testinator, Constants.Playstation.NorthPOVButton.getID()).whenPressed(new TurnToAngle2(drivetrain, shifter, 45));
    // new POVButton(testinator, Constants.Playstation.EastPOVButton.getID()).whenPressed(new TurnToAngle2(drivetrain, shifter, 90));
    // new POVButton(testinator, Constants.Playstation.WestPOVButton.getID()).whenPressed(new TurnToAngle2(drivetrain, shifter, -180));
    // new POVButton(testinator, Constants.Playstation.SouthPOVButton.getID()).whenPressed(new TurnToAngle2(drivetrain, shifter, -90));

    new POVButton(testinator, Constants.Playstation.NorthPOVButton.getID()).whenPressed(new TurnToAngleProfiled(drivetrain, 180));
    new POVButton(testinator, Constants.Playstation.EastPOVButton.getID()).whenPressed(new TurnToAngleProfiled(drivetrain, 90));
    new POVButton(testinator, Constants.Playstation.WestPOVButton.getID()).whenPressed(new TurnToAngleProfiled(drivetrain, -180));
    new POVButton(testinator, Constants.Playstation.SouthPOVButton.getID()).whenPressed(new TurnToAngleProfiled(drivetrain, -90));
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

    climber.setDefaultCommand(
      new Control(
        climber, 
        () -> operator.getRawAxis(Constants.Playstation.LeftYAxis.getID()), 
        () -> operator.getRawAxis(Constants.Playstation.RightYAxis.getID())
      )
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

  private void setupDashboard() {
    //? Drivetrain
    drivetrain.dashboard();
    //? Intake

    //? Horizontal Indexer

    //? Vertical Indexer

    //? Shifter

    //? Climber

  }

  public Command getAutonomousCommand() {
    //? Reset Sensors
    // drivetrain.resetEncoders();
    // drivetrain.resetGyro();
    String name = "work";
    drivetrain.resetOdometry(TrajectoryLoader.loadTrajectoryFromFile(name).getInitialPose());
    System.out.println(TrajectoryLoader.loadTrajectoryFromFile(name).getInitialPose());

    return standardRamseteCommand(name);

    // return new ParallelCommandGroup(
    //   new DefaultSetToHighGear(shifter),
    //   // test_fwd()
    //   // standardRamseteCommand("fwd")
    //   standardRamseteRevCommand("fwd")
      // new RamseteCommand(
      //   TrajectoryLoader.loadTrajectoryFromFile("u_curve_rev"),
      //   drivetrain::getPose,
      //   new RamseteController(),
      //   new SimpleMotorFeedforward(
      //     Constants.Drivetrain.kS,
      //     Constants.Drivetrain.kV,
      //     Constants.Drivetrain.kA
      //   ),
      //   Constants.Drivetrain.kDriveKinematics,
      //   drivetrain::getWheelSpeedsRev,
      //   new PIDController(1.1, 0.01, 0.15),
      //   new PIDController(1.5, 0.01, 0.05),
      //   drivetrain::tankDriveVoltsRev,
      //   drivetrain
      // )
    // );
  }


  // public Command getAutonomousCommand() {
  //   //? Reset Sensors
  //   drivetrain.resetEncoders();
  //   drivetrain.resetGyro();

  //   /*
  //   Trajectory trajectory = TrajectoryLoader.loadTrajectoryFromFile("Unnamed");
  //   RamseteCommand ramseteCommand = new RamseteCommand(
  //     trajectory,
  //     drivetrain::getPose,
  //     new RamseteController(2.0, 0.7), // 2.3
  //     new SimpleMotorFeedforward(
  //       Constants.Drivetrain.kS,
  //       Constants.Drivetrain.kV,
  //       Constants.Drivetrain.kA
  //     ),
  //     Constants.Drivetrain.kDriveKinematics,
  //     drivetrain::getWheelSpeeds,
  //     new PIDController(1.5, 0.01, 0.05),
  //     new PIDController(1.1, 0.01, 0.15),
  //     drivetrain::tankDriveVolts,
  //     drivetrain
  //   );
  //   */

  //   Position pos = positionChooser.getSelected();
  //   Goal goal = goalChooser.getSelected();

  //   if (pos == Position.Nothing) return new Nothing();
    
  //   else if (pos == Position.Left) { 
  //     if (goal == Goal.Safe) return new LeftAuton(drivetrain, intake, horizIndexer, vertIndexer, shooter);
  //     else if (goal == Goal.Ambitious) return test5Ball();
  //   }

  //   else if (pos == Position.Middle) return new MiddleAuton(drivetrain, intake, horizIndexer, vertIndexer, shooter); 

  //   else if (pos == Position.Right) { 
  //     if (goal == Goal.Safe) return new RightAuton(drivetrain, intake, horizIndexer, vertIndexer, shooter); 
  //     else if (goal == Goal.Ambitious) return test6Ball();
  //   }

  //   else return new Nothing();

  //   // return ramseteCommand.andThen(() -> drivetrain.tankDrive(0.0, 0.0));
  //   return new InstantCommand(() -> drivetrain.tankDrive(0.0, 0.0));
  // }

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

  public Command test5Ball() {
    return new DriveBackward(drivetrain, 0.5).withTimeout(1.85).andThen(
      new ParallelCommandGroup(
        new Align(drivetrain).withTimeout(3.0),
        new IndexAndShoot(intake, horizIndexer, vertIndexer, shooter)
      ).andThen(
        new RamseteCommand(
          TrajectoryLoader.loadTrajectoryFromFile("u_curve"),
          drivetrain::getPose,
          new RamseteController(),
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
        )
      ).andThen(
        new ParallelCommandGroup(
          new IntakeIntake(intake),
          new DriveForward(drivetrain, 0.5).withTimeout(1.1)
        ).withTimeout(1.2)
      ).andThen(
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
      ).andThen(
        new ParallelCommandGroup(
          new Align(drivetrain).withTimeout(3.0),
          new IndexAndShoot(intake, horizIndexer, vertIndexer, shooter)
        )
      ).andThen(() -> drivetrain.tankDrive(0.0, 0.0))
    );
  }

  public Command test6Ball() {
    return new ParallelCommandGroup(
      new Align(drivetrain).withTimeout(3.0),
      new IndexAndShoot(intake, horizIndexer, vertIndexer, shooter)
    ).andThen(
      new TurnToAngleProfiled(drivetrain, 180.0)
    ).andThen(
      new ParallelCommandGroup(
        new IntakeIntake(intake),
        new DriveForward(drivetrain, 0.5).withTimeout(2.5)
      ).withTimeout(2.6)
    ).andThen(
      new DriveBackward(drivetrain, 0.5).withTimeout(2.2)
    ).andThen(
      new TurnToAngleProfiled(drivetrain, 180.0)
    ).andThen(
      new ParallelCommandGroup(
        new Align(drivetrain).withTimeout(3.0),
        new IndexAndShoot(intake, horizIndexer, vertIndexer, shooter)
      )
    ).andThen(() -> drivetrain.tankDrive(0.0, 0.0));
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
      TrajectoryLoader.loadTrajectoryFromFile(name),
      drivetrain::getPose,
      new RamseteController(2.0, 0.7), // 2.3
      new SimpleMotorFeedforward(
        Constants.Drivetrain.kS,
        Constants.Drivetrain.kV,
        Constants.Drivetrain.kA
      ),
      Constants.Drivetrain.kDriveKinematics,
      drivetrain::getWheelSpeedsRev,
      new PIDController(1.5, 0.01, 0.15),
      new PIDController(1.5, 0.01, 0.05),
      drivetrain::tankDriveVoltsRev,
      drivetrain
    ).andThen(() -> drivetrain.arcadeDrive(0.0, 0.0));
  }

  public static Command getDriveStraight() {
    return d_driveStraight;
  }
}
