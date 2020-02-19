package frc.robot.commands.Macros.Tests;

import frc.robot.commands.Drivetrain.TankDrive;
import frc.robot.commands.Shifter.HoldSetToLowGear;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Shifter;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class TestDrivetrainAndShifter extends SequentialCommandGroup {

    public TestDrivetrainAndShifter(Drivetrain drivetrain, Shifter shifter) {
        super(
            new TankDrive(drivetrain, 0.5, 0.5).withTimeout(1.5),
            new ParallelCommandGroup(
                new HoldSetToLowGear(shifter),
                new TankDrive(drivetrain, 0.5, 0.5)
            ).withTimeout(1.5),
            new TankDrive(drivetrain, -0.5, -0.5).withTimeout(1.5),
            new ParallelCommandGroup(
                new HoldSetToLowGear(shifter),
                new TankDrive(drivetrain, -0.5, -0.5)
            ).withTimeout(1.5)
        );
    }

}