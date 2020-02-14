package frc.robot.commands.Macros;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Limelight.CorrectDistance;
import frc.robot.commands.Limelight.GyroAlign;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Limelight;

public class GyroAlignAndCorrectDist extends SequentialCommandGroup {

    public GyroAlignAndCorrectDist(Drivetrain drivetrain, Limelight limelight) {
        addCommands(
            new CorrectDistance(drivetrain, limelight),
            new GyroAlign(drivetrain, limelight)
        );
    }

}