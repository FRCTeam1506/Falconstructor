package frc.robot.commands.Drivetrain;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Drivetrain;

public class DriveStraight extends CommandBase {

    private static double m_targetHeading = 0.0;
    private static double m_rot, m_fwd;
    private Drivetrain m_drivetrain;

    public DriveStraight(Drivetrain drivetrain) {
        // super(
        //     new PIDController(
        //     Constants.Drivetrain.STABILIZATION_PID[0],
        //     Constants.Drivetrain.STABILIZATION_PID[1],
        //     Constants.Drivetrain.STABILIZATION_PID[2]
        //     ),
        //     drivetrain::getTurnRate,
        //     m_targetHeading = m_targetHeading + rot * 0.01,
        //     output -> drivetrain.arcadeDrive(-fwd, output),
        //     drivetrain
        m_drivetrain = drivetrain;
    }

    @Override
    public void initialize() {
    
    }

    @Override
    public void execute() {
        m_drivetrain.setTargetHeading(m_drivetrain.getTargetHeading() + RobotContainer.driver.getRawAxis(Constants.Playstation.RightXAxis.getID()) * 1.8);
        // m_targetHeading = m_targetHeading + RobotContainer.driver.getRawAxis(Constants.Playstation.RightXAxis.getID()) * 0.8;
        m_rot = RobotContainer.driver.getRawAxis(Constants.Playstation.RightXAxis.getID());
        m_fwd = RobotContainer.driver.getRawAxis(Constants.Playstation.LeftYAxis.getID());
        double error = m_drivetrain.getTargetHeading() - -m_drivetrain.getHeading();
        if(error > 180) { error = error - 360; }
        else if(error < -180) { error = error + 360; }
        // System.out.println(m_rot + " " + m_drivetrain.getTargetHeading() + " " + m_drivetrain.getHeading());
        // System.out.println(m_drivetrain.getTurnRate());
        // new PIDCommand(
        //     new PIDController(
        //         Constants.Drivetrain.STABILIZATION_PID[0],
        //         Constants.Drivetrain.STABILIZATION_PID[1],
        //         Constants.Drivetrain.STABILIZATION_PID[2]
        //     ),
        //     m_drivetrain::getHeading,
        //     m_targetHeading,
        //     output -> m_drivetrain.arcadeDrive(-m_fwd, output),
        //     m_drivetrain
        // );
        m_drivetrain.arcadeDrive(-m_fwd, 0.015 * error);
    }

    // @Override
    // public void execute() {
    //     m_targetHeading = m_targetHeading + m_rot * 0.01;
    // }
}