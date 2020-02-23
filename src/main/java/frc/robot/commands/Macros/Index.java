package frc.robot.commands.Macros;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.HorizIndexer.HorizIndex;
import frc.robot.commands.HorizIndexer.HorizIndexLeft;
import frc.robot.commands.Intake.IntakeIntake;
import frc.robot.commands.VertIndexer.VertIndex;
import frc.robot.subsystems.HorizIndexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.VertIndexer;

public class Index extends ParallelCommandGroup {

    public Index(HorizIndexer horizIndexer, VertIndexer vertIndexer, Intake intake) {
        super(
            new HorizIndex(horizIndexer),
            new VertIndex(vertIndexer),
            new IntakeIntake(intake, 0.1)
        );
    }

}