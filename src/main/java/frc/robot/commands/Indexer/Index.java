package frc.robot.commands.Indexer;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Indexer;

public class Index extends CommandBase {

    private final Indexer m_indexer;
    private final Double m_power;

    public Index(Indexer indexer, Double power) {
        m_indexer = indexer;
        m_power = power;
        addRequirements(indexer);
    }

    @Override
    public void execute() {
        m_indexer.index(m_power);
    }

}