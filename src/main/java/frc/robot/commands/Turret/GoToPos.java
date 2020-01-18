package frc.robot.commands.Turret;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Turret;

public class GoToPos extends CommandBase {
    // Declare Variables
    private final Turret m_turret;
    private final Double m_pos;

    public GoToPos(Turret subsystem, Double pos) {
        m_turret = subsystem;
        m_pos = pos;
        addRequirements(m_turret);
    }

    @Override
    public void execute() {
        m_turret.goToPos(m_pos);
    }
}