package frc.robot.commands.Macros;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.Shooter.Shoot;
import frc.robot.subsystems.HorizIndexer;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.VertIndexer;

public class IndexAndShoot extends ParallelCommandGroup {

    public IndexAndShoot(HorizIndexer horizIndexer, VertIndexer vertIndexer, Shooter shooter) {

        super(
            new Shoot(shooter, 22000.0),
            new SequentialCommandGroup(
                new WaitCommand(2.0),
                new Index(horizIndexer, vertIndexer)
            )
        );

    }

}