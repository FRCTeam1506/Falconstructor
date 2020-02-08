package frc.robot.commands.Macros;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.Limelight.Align;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;

public class AimAndShoot extends ParallelCommandGroup {
    
    public AimAndShoot(Drivetrain drivetrain, Limelight limelight, Intake intake, Shooter shooter) {
        addCommands(
            new Align(drivetrain, limelight),
            new IntakeAndShoot(intake, shooter)
        );
        addRequirements(limelight, intake, shooter);
    }
}