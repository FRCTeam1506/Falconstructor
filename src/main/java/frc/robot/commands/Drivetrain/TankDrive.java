package frc.robot.commands.Drivetrain;

import frc.robot.subsystems.Drivetrain;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class TankDrive extends CommandBase {

  private final Drivetrain m_drivetrain;
  private final Double m_left_power;
  private final Double m_right_power;

  public TankDrive(Drivetrain drivetrain, Double left, Double right) {
    m_drivetrain = drivetrain;
    m_left_power = left;
    m_right_power = right;
    addRequirements(m_drivetrain);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    m_drivetrain.tankDrive(m_left_power, m_right_power);
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
