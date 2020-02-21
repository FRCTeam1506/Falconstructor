package frc.robot.commands.Macros.Tests;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.HorizIndexer.HorizIndexLeft;
import frc.robot.commands.HorizIndexer.HorizIndexRight;
import frc.robot.commands.Intake.ExtendAndIntake;
import frc.robot.commands.Shooter.Shoot;
import frc.robot.commands.VertIndexer.VertIndex;
import frc.robot.subsystems.HorizIndexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.VertIndexer;

public class TestMechanisms extends SequentialCommandGroup {

    public TestMechanisms(Intake intake, HorizIndexer horizIndexer, VertIndexer vertIndexer, Shooter shooter) {
        super(
            new ExtendAndIntake(intake).withTimeout(1.5),
            new HorizIndexLeft(horizIndexer).withTimeout(1.5),
            new HorizIndexRight(horizIndexer).withTimeout(1.5),
            new VertIndex(vertIndexer).withTimeout(1.5),
            new Shoot(shooter, 22000.0).withTimeout(1.5)
        );
    }

}