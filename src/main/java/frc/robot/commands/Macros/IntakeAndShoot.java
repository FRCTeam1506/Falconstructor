package frc.robot.commands.Macros;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.Intake.IntakeDouble;
import frc.robot.commands.Shooter.FullSend;
import frc.robot.commands.Shooter.FullSend2;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class IntakeAndShoot extends ParallelCommandGroup {

    public IntakeAndShoot(Intake intake, Shooter shooter) {
        addCommands(
            new FullSend(shooter),
            new SequentialCommandGroup(
                new WaitCommand(1.0),
                new IntakeDouble(intake, -0.85)
            )
        );
    }
}