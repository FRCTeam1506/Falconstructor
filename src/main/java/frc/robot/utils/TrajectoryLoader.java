package frc.robot.utils;

import java.io.IOException;
import java.nio.file.Path;
import java.util.ArrayList;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Transform2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.util.Units;
import frc.robot.Constants;

public class TrajectoryLoader {

    public static Trajectory loadTrajectoryFromFile(String filename) {
        Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve("output/" + filename + ".wpilib.json");
        Trajectory trajectory = null;

        try {
            System.out.println(trajectoryPath);
            trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
        } catch (IOException e) {
            DriverStation.reportError("Unable to open trajectory: " + filename, e.getStackTrace());
        }

        return trajectory;
    }

    public static Pose2d getInitialPoseReversed(Trajectory trajectory) {
        return trajectory.getInitialPose().transformBy(new Transform2d(new Translation2d(0, 0), new Rotation2d(Math.PI)));
    }

    //! Returns work trajectory
    // todo: accept json as parameter -> parse start, interior, and end points -> return trajectory
    public static Trajectory createReverseTrajectory(String name) {
        //? Work Trajectory
        var start = new Pose2d(new Translation2d(1.7898031746491527, -1.446), new Rotation2d(0.0));
        var end = new Pose2d(new Translation2d(4.1883449637127175, -1.4459393814906376), new Rotation2d(3.456127088688585e-14));

        var interiorWaypoints = new ArrayList<Translation2d>();
        interiorWaypoints.add(new Translation2d(3.093649398178689, -1.44599902693677));

        TrajectoryConfig config = new TrajectoryConfig(Constants.Drivetrain.MAX_VELOCITY, Constants.Drivetrain.MAX_ACCELERATION);
        config.setReversed(true);

        return TrajectoryGenerator.generateTrajectory(
            start,
            interiorWaypoints,
            end,
            config
        );
    }

}
