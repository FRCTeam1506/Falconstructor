package frc.robot.commands.Drivetrain;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class SetPipeline extends CommandBase {

    private final Drivetrain m_drivetrain;
    private final Drivetrain.Piplelines m_pipeline;

    public SetPipeline(Drivetrain drivetrain, Drivetrain.Piplelines pipleline) {
        m_drivetrain = drivetrain;
        m_pipeline = pipleline;
        addRequirements(m_drivetrain);
    }

    @Override
    public void initialize() {
        m_drivetrain.setPipeline(m_pipeline);
    }

}