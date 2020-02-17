package frc.robot.commands.Auto;

import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Drivetrain.DriveTrajectory;
import frc.robot.subsystems.Drivetrain;
import frc.robot.utils.TrajectoryLoader;

public class TestAuto extends SequentialCommandGroup {

  public TestAuto(Drivetrain drivetrain) {
    Trajectory trajectory = TrajectoryLoader.loadTrajectoryFromFile("v2");
    addCommands(
      new DriveTrajectory(drivetrain, trajectory, false)
    );
  }
}
