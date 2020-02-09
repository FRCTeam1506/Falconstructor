package frc.robot.commands.Macros;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.Indexer.Index;
import frc.robot.commands.Intake.IntakeDouble;
import frc.robot.commands.Shooter.FullSend;
import frc.robot.commands.Shooter.FullSend2;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class IntakeAndShoot extends ParallelCommandGroup {

    public IntakeAndShoot(Intake intake, Indexer indexer, Shooter shooter) {
        addCommands(
            new FullSend2(shooter),
            new Index(indexer, 0.5),
            new SequentialCommandGroup(
                new WaitCommand(1.0),
                new IntakeDouble(intake, -0.85)
            )
        );
    }
}