package frc.robot.commands.Drivetrain;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.Shifter.DefaultSetToLowGear;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Shifter;

public class Align2 extends ParallelCommandGroup {

    public Align2(Drivetrain drivetrain, Shifter shifter) {
        super(
            new Align(drivetrain),
            new DefaultSetToLowGear(shifter)
        );
    }

    public Align2(Drivetrain drivetrain, Shifter shifter, double targetXError) {
        super(
            new Align(drivetrain, targetXError),
            new DefaultSetToLowGear(shifter)
        );
    }

}