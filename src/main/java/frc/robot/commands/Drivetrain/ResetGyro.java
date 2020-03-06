package frc.robot.commands.Drivetrain;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class ResetGyro extends CommandBase {

    private final Drivetrain m_drivetrain;

    public ResetGyro(Drivetrain drivetrain) {
        m_drivetrain = drivetrain;
        addRequirements(m_drivetrain);
    }

    @Override
    public void initialize() {
        m_drivetrain.resetGyro();
    }

    @Override
    public boolean isFinished() {
        return true;
    }

}