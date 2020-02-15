package frc.robot.commands.Macros;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Drivetrain.DriveToDist;
import frc.robot.commands.Drivetrain.TurnToAngle;
import frc.robot.commands.Shooter.FullSend2;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;

public class TrenchFar extends SequentialCommandGroup {

    public TrenchFar(Drivetrain drivetrain, Limelight limelight, Shooter shooter, Intake intake, Indexer indexer) {
        super(
            new ParallelCommandGroup(
                new FullSend2(shooter),
                new DriveToDist(drivetrain, limelight, 5000.0)
            ),
            new TurnToAngle(drivetrain, limelight),
            new IndexerAndIntake(intake, indexer)
        );
    }

}