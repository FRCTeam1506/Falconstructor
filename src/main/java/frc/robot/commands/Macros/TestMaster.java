package frc.robot.commands.Macros;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Macros.Tests.TestDrivetrainAndShifter;
import frc.robot.commands.Macros.Tests.TestIndexer;
import frc.robot.commands.Macros.Tests.TestIntake;
import frc.robot.commands.Macros.Tests.TestShooter;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.HorizIndexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shifter;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.VertIndexer;

public class TestMaster extends SequentialCommandGroup {

    public TestMaster(Drivetrain drivetrain, Shifter shifter, Intake intake, HorizIndexer horizIndexer, VertIndexer vertIndexer, Shooter shooter) {
        super(
            new TestDrivetrainAndShifter(drivetrain, shifter),
            new TestIntake(intake),
            new TestIndexer(horizIndexer, vertIndexer)
            // new TestShooter(shooter)
        );
    }

}