package frc.robot.commands.Shooter;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooter;

public class Shoot extends CommandBase {
    // Declare Variables
    private final Shooter m_shooter;
    private final DoubleSupplier m_power;

    public Shoot(Shooter subsystem, DoubleSupplier power) {
        m_shooter = subsystem;
        m_power = power;
        addRequirements(m_shooter);
    }

    @Override
    public void execute() {
        m_shooter.shoot(m_power.getAsDouble());
    }
}