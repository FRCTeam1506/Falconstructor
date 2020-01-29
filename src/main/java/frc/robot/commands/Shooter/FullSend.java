package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooter;

public class FullSend extends CommandBase {
    // Declare Variables
    private final Shooter m_shooter;

    public FullSend(Shooter subsystem) {
        m_shooter = subsystem;
        addRequirements(m_shooter);
    }

    @Override
    public void execute() {
        m_shooter.fullSend();
    }
}