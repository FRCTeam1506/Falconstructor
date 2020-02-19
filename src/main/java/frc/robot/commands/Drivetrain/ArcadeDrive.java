package frc.robot.commands.Drivetrain;

import frc.robot.subsystems.Drivetrain;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class ArcadeDrive extends CommandBase {

  private final Drivetrain m_drivetrain;
  private final DoubleSupplier m_forward;
  private final DoubleSupplier m_rotation;

  public ArcadeDrive(Drivetrain drivetrain, DoubleSupplier fwd, DoubleSupplier rot) {
    m_drivetrain = drivetrain;
    m_forward = fwd;
    m_rotation = rot;
    addRequirements(m_drivetrain);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    m_drivetrain.arcadeDrive(m_forward.getAsDouble(), m_rotation.getAsDouble());
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
