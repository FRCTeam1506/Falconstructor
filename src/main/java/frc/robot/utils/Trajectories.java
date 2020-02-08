// package frc.robot.utils;

// import java.util.Map;

// import edu.wpi.first.wpilibj.Filesystem;
// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.InstantCommand;
// import edu.wpi.first.wpilibj2.command.SelectCommand;
// import frc.robot.RobotContainer;
// import frc.robot.commands.Trajectory.RunTrajectory;

// public class Trajectories {

//     private String todo = "todo";

//     public String trajectoryJSON;

//     private enum TrajectorySelector {
//         WIGGLE,
//     }

//     private TrajectorySelector select() {
//         TrajectorySelector choice;
//         trajectoryJSON = "paths/";
//         switch (todo) {
//             case "":
//                 choice = TrajectorySelector.WIGGLE;
//                 trajectoryJSON += "1auto";
//                 break;
        
//             default:
//                 choice = TrajectorySelector.WIGGLE;
//                 trajectoryJSON += "1auto";
//                 break;

//             trajectoryJSON += ".wpilib.json";
//         }
//         return choice;
//     }

//     private final Command trajectorySelector = 
//         new SelectCommand(
//             Map.ofEntries(
//                 entry(TrajectorySelector.WIGGLE, new InstantCommand(new RunTrajectory(RobotContainer.drivetrain, trajectoryJSON)))
//             ),
//             this::select
//         );
// }