package frc.robot.commands.Macros.Tests;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.Intake.Extend;
import frc.robot.commands.Intake.ExtendAndIntake;
import frc.robot.commands.Intake.IntakeIntake;
import frc.robot.subsystems.Intake;

public class TestIntake extends SequentialCommandGroup {

    public TestIntake(Intake intake) {
        super(
            new ExtendAndIntake(intake).withTimeout(2.0)
        );
    }

}