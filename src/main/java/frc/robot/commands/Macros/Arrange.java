package frc.robot.commands.Macros;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;

import frc.robot.commands.Limelight.CorrectDistance;
import frc.robot.commands.Limelight.GyroAlign;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Limelight;

public class Arrange extends CommandBase {

    private final Drivetrain m_drivetrain;
    private final Limelight m_limelight;
    private final Double m_targetDistance;

    private boolean finished = false;

    public Arrange(Drivetrain drivetrain, Limelight limelight, Double targetDistance) {
        m_drivetrain = drivetrain;
        m_limelight = limelight;
        m_targetDistance = targetDistance;
        addRequirements(m_drivetrain, m_limelight);
    }

    @Override
    public void initialize() {
        m_limelight.setTargetDistance(m_targetDistance);
    }

    @Override
    public void execute() {
        if(m_limelight.isTargetFound()) {
            double currentHeading = m_drivetrain.getHeading();
            double cameraXError = m_limelight.getX();
            double currentDistance = m_limelight.getDistance();
            double distanceError = currentDistance - m_limelight.getTargetDistance();
            if(!((distanceError > -100) && (distanceError < 100))) {
                System.out.println("Distance Error " + distanceError);
                distanceError = distanceError / 4000.0;
                if(distanceError > 0) {
                    System.out.println("Power Output " + distanceError);
                    // m_drivetrain.regArcadeDrive(-distanceError * 0.5, 0.0);
                    m_drivetrain.fwd(-distanceError * 0.5);
                } else {
                    // m_drivetrain.regArcadeDrive(0.11, 0.0);
                    m_drivetrain.fwd(0.2);
                }
            } else {
                finished = true;
            }
            if(m_limelight.isRefreshed) {
                System.out.println("Refreshed");
                // double targetHeading = currentHeading + (100.0 / 2.0) * cameraXError; // pixels / degrees
                double targetHeading = currentHeading - cameraXError;
                System.out.println("Target Heading " + targetHeading);
                m_drivetrain.setTargetHeading(targetHeading);
            }
            System.out.println("Current Heading " + currentHeading);
            double headingError = m_drivetrain.getTargetHeading() - currentHeading;
            System.out.println("Heading Error " + headingError);
            if(!((headingError > -1) && (headingError < 1))) {
                // headingError = Math.IEEEremainder(headingError, 1);
                headingError = headingError / 180.0;
                System.out.println("Motor output: " + headingError);
                // m_drivetrain.regArcadeDrive(0.0, -headingError * (2.4 + 0.8 * 2)); // FOR KOBE AND GIGI
                m_drivetrain.rot(-headingError * (2.4 + 0.8 * 2));
            }   
        }
    }
    @Override
    public boolean isFinished() {
        return finished;
    }
}