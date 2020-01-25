package frc.robot.commands.Auto;

import java.io.IOException;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.Constants;
import frc.robot.commands.Drivetrain.ProfileFollowerUpdate;
import frc.robot.subsystems.Drivetrain;

public class TestDrive extends CommandBase {

    private final String path = Constants.General.DEPLOY_PATH + "/profiles/test/test";

    public TestDrive(Drivetrain drivetrain) {
        addRequirements(drivetrain);
    }

    @Override
    public void execute() {
        System.out.println("Test auton running !!!");
        try {
            new ProfileFollowerUpdate(path + "_left.csv", path + "_right.csv");
        } catch (IOException e) {
            e.printStackTrace();
        }
    }
}