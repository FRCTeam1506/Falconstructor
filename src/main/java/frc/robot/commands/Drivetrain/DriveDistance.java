package frc.robot.commands.Drivetrain;

import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.TrapezoidProfileCommand;
import frc.robot.subsystems.Drivetrain;

public class DriveDistance extends TrapezoidProfileCommand {

    public DriveDistance(Drivetrain drivetrain, double meters) {
        super(
            new TrapezoidProfile(
                new TrapezoidProfile.Constraints(1.5, 0.25),
                new TrapezoidProfile.State(
                    meters,
                    0.0
                )
            ),
            setpointState -> drivetrain.setDriveStates(setpointState, setpointState),
            drivetrain
        );
        drivetrain.resetEncoders();
    }

}