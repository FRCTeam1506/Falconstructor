package frc.robot.commands.Macros.Tests;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.HorizIndexer.HorizIndex;
import frc.robot.commands.HorizIndexer.HorizIndexRev;
import frc.robot.commands.VertIndexer.VertIndex;
import frc.robot.commands.VertIndexer.VertIndexRev;
import frc.robot.subsystems.HorizIndexer;
import frc.robot.subsystems.VertIndexer;

public class TestIndexer extends SequentialCommandGroup {

    public TestIndexer(HorizIndexer horizIndexer, VertIndexer vertIndexer) {
        super(
            new ParallelCommandGroup(
                new HorizIndex(horizIndexer),
                new VertIndex(vertIndexer)
            ).withTimeout(1.5),
            new WaitCommand(0.1),
            new ParallelCommandGroup(
                new HorizIndexRev(horizIndexer),
                new VertIndexRev(vertIndexer)
            ).withTimeout(1.5)
        );
    }
    
}