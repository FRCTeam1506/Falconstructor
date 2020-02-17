package frc.robot.commands.Drivetrain;

import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class DriveTrajectory extends CommandBase {

    private final Drivetrain drivetrain;
    private final Trajectory trajectory;
    private final boolean reversed;

    public DriveTrajectory(Drivetrain drivetrain, Trajectory trajectory, boolean reversed) {
        this.drivetrain = drivetrain;
        this.trajectory = trajectory;
        this.reversed = reversed;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(drivetrain);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        System.out.println("Running " + this.getClass().toString());
        
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        drivetrain.driveTrajectory(trajectory, reversed);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {

    }
}
