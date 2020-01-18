package frc.robot.commands.Drivetrain;

import frc.robot.RobotContainer;

import java.io.IOException;

import edu.wpi.first.wpilibj2.command.CommandBase;
import jaci.pathfinder.Waypoint;
import jaci.pathfinder.followers.EncoderFollower;

public class WaypointFollower extends CommandBase {
	Waypoint[] path;
	EncoderFollower[] followers;

	public WaypointFollower(Waypoint[] path) throws IOException {
		this.path = path;
	    followers = RobotContainer.drivetrain.initPath(path);
	}

	@Override
	public void initialize() {
		RobotContainer.drivetrain.resetForPath();
		RobotContainer.drivetrain.executePath(followers, false);
	}

	@Override
	public void execute() {
		RobotContainer.drivetrain.executePath(followers, false);
	}
}
