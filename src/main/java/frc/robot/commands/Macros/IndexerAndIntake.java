package frc.robot.commands.Macros;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.Indexer.Index;
import frc.robot.commands.Intake.IntakeDouble;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;

public class IndexerAndIntake extends ParallelCommandGroup {

    public IndexerAndIntake(Intake intake, Indexer indexer) {
        super(
            new Index(indexer, 0.5),
            new IntakeDouble(intake, -0.85)
        );
    }

}