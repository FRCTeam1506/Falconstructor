package frc.robot.commands.Drivetrain;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class VelocityDrive extends CommandBase {

    private final Drivetrain m_drivetrain;
    private final DoubleSupplier m_fwd;
    private final DoubleSupplier m_rot;

    public VelocityDrive(Drivetrain drivetrain, DoubleSupplier fwd, DoubleSupplier rot) {
        m_drivetrain = drivetrain;
        m_fwd = fwd;
        m_rot = rot;
        addRequirements(m_drivetrain);
    }

    @Override
    public void execute() {
        m_drivetrain.velocityArcadeDrive(-m_fwd.getAsDouble(), m_rot.getAsDouble());
    }

}