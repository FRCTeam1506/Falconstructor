package frc.robot.commands.Climber;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climber;

public class Control extends CommandBase {

    private final Climber m_climber;
    private final DoubleSupplier m_l, m_r;

    public Control(Climber climber, DoubleSupplier l, DoubleSupplier r) {
        m_climber = climber;
        m_l = l;
        m_r = r;
        addRequirements(m_climber);
    }

    @Override
    public void execute() {
        m_climber.controlClimber(m_l.getAsDouble(), m_r.getAsDouble());
        // else m_climber.controlClimber(0.0, 0.0);
    }

}