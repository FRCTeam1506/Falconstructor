package frc.robot.commands.Shooter;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooter;

public class ShootManual extends CommandBase {

    private final Shooter m_shooter;
    private final DoubleSupplier m_velocity;

    public ShootManual(Shooter shooter, DoubleSupplier velocity) {
        m_shooter = shooter;
        m_velocity = velocity;
        addRequirements(m_shooter);
    }

    @Override
    public void execute() {
        m_shooter.shoot(m_velocity.getAsDouble());
    }

}