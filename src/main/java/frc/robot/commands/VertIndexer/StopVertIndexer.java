package frc.robot.commands.VertIndexer;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.VertIndexer;

public class StopVertIndexer extends CommandBase {

    private final VertIndexer m_vertIndexer;

    public StopVertIndexer(VertIndexer vertIndexer) {
        m_vertIndexer = vertIndexer;
        addRequirements(vertIndexer);
    }

    @Override
    public void execute() {
        m_vertIndexer.stop();
    }

}