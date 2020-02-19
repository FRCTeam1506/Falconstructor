package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Intake;

public class ExtendAndIntake extends ParallelCommandGroup {

    public ExtendAndIntake(Intake intake) {
        super(
            new Extend(intake),
            new SequentialCommandGroup(
                new WaitCommand(0.5),
                new IntakeIntake(intake)
            )
        );
    }
    
}