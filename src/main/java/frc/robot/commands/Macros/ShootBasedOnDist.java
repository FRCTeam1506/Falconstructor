package frc.robot.commands.Macros;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Shooter;

public class ShootBasedOnDist extends CommandBase {

    private final Drivetrain m_drivetrain;
    private final Shooter m_shooter;

    public ShootBasedOnDist(Drivetrain drivetrain, Shooter shooter) {
        m_drivetrain = drivetrain;
        m_shooter = shooter;
        addRequirements(drivetrain, shooter);
    }

    @Override
    public void execute() {
        m_shooter.shootVelocity(m_drivetrain.calculateDistance());
    }
}