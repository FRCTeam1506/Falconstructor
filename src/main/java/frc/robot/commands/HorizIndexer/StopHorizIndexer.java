package frc.robot.commands.HorizIndexer;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.HorizIndexer;

public class StopHorizIndexer extends CommandBase {

    private final HorizIndexer m_horizIndexer;

    public StopHorizIndexer(HorizIndexer horizIndexer) {
        m_horizIndexer = horizIndexer;
        addRequirements(horizIndexer);
    }

    @Override
    public void execute() {
        m_horizIndexer.stop();
    }

}