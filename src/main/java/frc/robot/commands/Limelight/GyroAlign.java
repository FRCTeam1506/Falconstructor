package frc.robot.commands.Limelight;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Drivetrain;

public class GyroAlign extends CommandBase {

    private final Drivetrain m_drivetrain;
    private final Limelight m_limelight;

    public GyroAlign(Drivetrain drivetrain, Limelight limelight) {
        m_drivetrain = drivetrain;
        m_limelight = limelight;
        addRequirements(m_drivetrain, m_limelight);
    }

    @Override
    public void execute() {
        double currentHeading = m_drivetrain.getHeading();
        double cameraXError = m_limelight.getX();
        double distance = m_limelight.getDistance();
        if(m_limelight.isRefreshed) {
            double targetHeading = currentHeading + 100.0 / 2.0; // pixels / degrees
            m_drivetrain.setTargetHeading(targetHeading);
        }
        double headingError = m_drivetrain.getTargetHeading() - currentHeading;
        m_drivetrain.arcadeDrive(0.0, -0.01 * headingError);
    }

}