package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooter;

public class Shoot extends CommandBase {

    private final Shooter m_shooter;
    private final Double m_velocity;

    public Shoot(Shooter shooter) {
        m_shooter = shooter;
        m_velocity = 25000.0;
        addRequirements(m_shooter);
    }

    public Shoot(Shooter shooter, Double velocity) {
        m_shooter = shooter;
        m_velocity = velocity;
        addRequirements(m_shooter);
    }

    @Override
    public void execute() {
        m_shooter.shootVelocity(m_velocity);
    }

}