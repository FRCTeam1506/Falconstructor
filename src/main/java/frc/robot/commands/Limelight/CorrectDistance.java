package frc.robot.commands.Limelight;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Limelight;

public class CorrectDistance extends CommandBase {

    private boolean finished = false;

    private final Drivetrain m_drivetrain;
    private final Limelight m_limelight;
    private final Double m_targetDistance;

    public CorrectDistance(Drivetrain drivetrain, Limelight limelight, Double targetDistance) {
        m_drivetrain = drivetrain;
        m_limelight = limelight;
        m_targetDistance = targetDistance;
    }

    public CorrectDistance(Drivetrain drivetrain, Limelight limelight) {
        m_drivetrain = drivetrain;
        m_limelight = limelight;
        m_targetDistance = 1234.0;
    }

    @Override
    public void initialize() {
        m_limelight.setTargetDistance(m_targetDistance);
    }

    @Override
    public void execute() {
        if(m_limelight.isTargetFound()) {
            double currentDistance = m_limelight.getDistance();
            double distanceError = currentDistance - m_limelight.getTargetDistance();
            System.out.println("Current Distance " + currentDistance);
            if(!((distanceError > -100) && (distanceError < 100))) {
                System.out.println("Distance Error " + distanceError);
                distanceError = distanceError / 4000.0;
                if(distanceError > 0) {
                    System.out.println("Power Output " + distanceError);
                    m_drivetrain.regArcadeDrive(-distanceError * 0.5, 0.0);
                }
            } else {
                finished = true;
            }
        }
    }
    @Override
    public boolean isFinished() {
        return finished;
    }

}