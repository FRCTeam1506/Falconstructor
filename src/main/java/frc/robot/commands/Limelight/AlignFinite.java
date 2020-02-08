package frc.robot.commands.Limelight;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Limelight;

public class AlignFinite extends CommandBase {

    private final Drivetrain drivetrain;
    private final Limelight limelight;
    
    public AlignFinite(Drivetrain m_drivetrain, Limelight m_limelight) {
        drivetrain = m_drivetrain;
        limelight = m_limelight;
        addRequirements(drivetrain, limelight);
    }

    @Override
    public void initialize() {
        if(limelight.getPipeline() != 5) {
            limelight.setPipeline(5);
        }
    }

    // add while loop
    @Override
    public void execute() {
        System.out.println("Executing ...");
        if(limelight.isTargetFound()) {
            Double variation = limelight.getX();
            // if(Math.abs(variation) < 0.1) RobotContainer.setAlign(true);
            if(!((variation > -Constants.Limelight.THRESHOLD) && (variation < Constants.Limelight.THRESHOLD))) {
                RobotContainer.setAlign(false);
                Double rot = variation * Constants.Limelight.CONVERSION;
                System.out.println("Rot: " + rot);
                if(Math.abs(rot) < 0.2) {
                    drivetrain.arcadeDrive(0.0, -rot * 1.15);
                } else if(Math.abs(rot) < 0.1) {
                    drivetrain.arcadeDrive(0.0, -rot * 1.2);
                }
                else {
                    drivetrain.arcadeDrive(0.0, -rot * Constants.Limelight.LIMIT);
                }
            }
        }
    }
    @Override
    public boolean isFinished() {
        return limelight.isAligned();
    }
}