package frc.robot.commands.Limelight;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Limelight;

public class StandardAlignAndSeek extends CommandBase {

    private final Drivetrain drivetrain;
    private final Limelight limelight;
    
    public StandardAlignAndSeek(Drivetrain m_drivetrain, Limelight m_limelight) {
        drivetrain = m_drivetrain;
        limelight = m_limelight;
        addRequirements(drivetrain, limelight);
    }

    @Override
    public void initialize() {
        if(limelight.getPipeline() != 5) {
            limelight.setPipeline(Limelight.Piplelines.NearTargeting);
        }
    }

    @Override
    public void execute() {
        System.out.println("[c] " + this.getClass().toString() + " executing ...");

        if(limelight.isTargetFound()) {
            Double variation = limelight.getX();
            Double error = -variation;
            Double steeringAdjust = 0.0;
            
            if(variation > Constants.Limelight.THRESHOLD) {
                steeringAdjust = Constants.Limelight.kP * error - Constants.Limelight.MIN_PWR;
            } else if(variation < Constants.Limelight.THRESHOLD) {
                steeringAdjust = Constants.Limelight.kP * error + Constants.Limelight.MIN_PWR;
            }
            drivetrain.arcadeDrive(0.0, -steeringAdjust);
        } else {
            drivetrain.arcadeDrive(0.0, -0.2);
        }
    }
}