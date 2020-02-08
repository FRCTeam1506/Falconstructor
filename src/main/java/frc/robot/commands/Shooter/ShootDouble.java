package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooter;

public class ShootDouble extends CommandBase {
    // Declare Variables
    private final Shooter m_shooter;
    private final Double m_power;

    public ShootDouble(Shooter subsystem, Double power) {
        m_shooter = subsystem;
        m_power = power;
        addRequirements(m_shooter);
    }

    @Override
    public void execute() {
        m_shooter.shoot(m_power);
    }
}