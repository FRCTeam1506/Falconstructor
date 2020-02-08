package frc.robot.commands.Limelight;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.RobotContainer;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Limelight;

public class Idle extends CommandBase {

    private final Drivetrain drivetrain;
    private final Limelight limelight;

    public Idle(Drivetrain m_drivetrain, Limelight m_limelight) {
        drivetrain = m_drivetrain;
        limelight = m_limelight;
        addRequirements(drivetrain, limelight);
    }

    @Override
    public void initialize() {
        RobotContainer.drivetrain.arcadeDrive(0.0, 0.0);
    }

    @Override
    public void execute() {
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}