package frc.robot.commands.Limelight;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Limelight;

public class Align extends CommandBase {

    private final Drivetrain drivetrain;
    private final Limelight limelight;
    
    public Align(Drivetrain m_drivetrain, Limelight m_limelight) {
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

    @Override
    public void execute() {
        System.out.println("Executing ...");
        if(limelight.isTargetFound()) {
            Double variation = limelight.getX();
            if(!((variation > -Constants.Limelight.THRESHOLD) && (variation < Constants.Limelight.THRESHOLD))) {
                Double rot = variation * Constants.Limelight.CONVERSION;
                System.out.println("Rot: " + rot);
                if(Math.abs(rot) < 0.5) {
                    drivetrain.arcadeDrive(0.0, -rot * 1.17);
                }
                else if(Math.abs(rot) < 0.2) {
                    drivetrain.arcadeDrive(0.0, -rot * 1.15);
                } else if(Math.abs(rot) < 0.1) {
                    drivetrain.arcadeDrive(0.0, -rot * 1.1);
                }
                else {
                    drivetrain.arcadeDrive(0.0, -rot * Constants.Limelight.LIMIT);
                }
            }
        }
    }
}