package frc.robot.commands.Macros.Tests;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Shooter.Shoot;
import frc.robot.subsystems.Shooter;

public class TestShooter extends SequentialCommandGroup {

    public TestShooter(Shooter shooter) {
        super(
            new Shoot(shooter).withTimeout(2.0)
        );
    }

}