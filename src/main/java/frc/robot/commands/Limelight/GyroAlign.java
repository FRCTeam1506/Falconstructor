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
    }

    @Override
    public void execute() {
        if(m_limelight.isTargetFound()) {
            double currentHeading = m_drivetrain.getHeading();
            double cameraXError = m_limelight.getX();
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
                m_drivetrain.regArcadeDrive(0.0, -headingError * (2.4 + 0.8 * 2)); // FOR KOBE
            }   
        }
    }
}