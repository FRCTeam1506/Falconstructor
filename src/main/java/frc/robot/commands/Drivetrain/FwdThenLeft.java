package frc.robot.commands.Drivetrain;

// Java Imports
import java.io.IOException;

// WPILIB Imports
import edu.wpi.first.wpilibj2.command.CommandBase;

// Project Imports
import frc.robot.Constants;
import frc.robot.commands.Drivetrain.ProfileFollowerUpdate;
import frc.robot.subsystems.Drivetrain;

public class FwdThenLeft extends CommandBase {

    private final String path = Constants.General.DEPLOY_PATH + "/profiles/right/fwd_left_turn";

    public FwdThenLeft(final Drivetrain drivetrain) {
        addRequirements(drivetrain);
    }

    @Override
    public void execute() {
        System.out.println("Fwd then left auton running !!!");
        try {
            new ProfileFollowerUpdate(path + "_left.csv", path + "_right.csv");
            // new WaypointFollower(Paths.fwdThenLeft);
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

}