package frc.robot.commands.VertIndexer;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.VertIndexer;

public class VertIndexRev extends CommandBase {

    private final VertIndexer m_vertIndexer;

    public VertIndexRev(VertIndexer vertIndexer) {
        m_vertIndexer = vertIndexer;
        addRequirements(m_vertIndexer);
    }

    @Override
    public void execute() {
        m_vertIndexer.down();
    }
}