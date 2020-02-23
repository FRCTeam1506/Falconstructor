package frc.robot.commands.Macros;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.HorizIndexer.HorizIndexRev;
import frc.robot.commands.Intake.IntakeIntake;
import frc.robot.commands.VertIndexer.VertIndexRev;
import frc.robot.subsystems.HorizIndexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.VertIndexer;

public class Unjam extends ParallelCommandGroup {

    public Unjam(HorizIndexer horizIndexer, VertIndexer vertIndexer, Intake intake) {
        super(
            new HorizIndexRev(horizIndexer),
            new VertIndexRev(vertIndexer),
            new IntakeIntake(intake, 0.1)
        );
    }

}