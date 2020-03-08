package frc.robot.commands.Macros;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.HorizIndexer.HorizIndexRevCycle;
import frc.robot.commands.Intake.IntakeIntake;
import frc.robot.commands.Shooter.Shoot;
import frc.robot.subsystems.HorizIndexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.VertIndexer;

public class IndexAndShoot extends ParallelCommandGroup {

    public IndexAndShoot(Intake intake, HorizIndexer horizIndexer, VertIndexer vertIndexer, Shooter shooter) {
        super(
            new Shoot(shooter, 24000.0),
            new SequentialCommandGroup(
                // new HorizIndexRevCycle(horizIndexer),
                new WaitCommand(1.5),
                new Index(horizIndexer, vertIndexer, intake)
            )
        );
    }

    public IndexAndShoot(Intake intake, HorizIndexer horizIndexer, VertIndexer vertIndexer, Shooter shooter, Double waitTimeSeconds) {
        super(
            new Shoot(shooter, 24000.0),
            new SequentialCommandGroup(
                // new HorizIndexRevCycle(horizIndexer),
                new WaitCommand(waitTimeSeconds),
                new Index(horizIndexer, vertIndexer, intake)
            )
        );
    }

}