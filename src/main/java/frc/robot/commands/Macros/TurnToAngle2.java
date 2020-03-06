package frc.robot.commands.Macros;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Drivetrain.ResetGyro;
import frc.robot.commands.Drivetrain.TurnToAngle;
import frc.robot.commands.Shifter.DefaultSetToHighGear;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Shifter;

public class TurnToAngle2 extends ParallelCommandGroup {

    public TurnToAngle2(Drivetrain drivetrain, Shifter shifter, double targetAngle) {
        super(
            new DefaultSetToHighGear(shifter),
            new SequentialCommandGroup(
                new ResetGyro(drivetrain),
                new TurnToAngle(drivetrain, targetAngle),
                new ResetGyro(drivetrain)
            )
        );
    }

}