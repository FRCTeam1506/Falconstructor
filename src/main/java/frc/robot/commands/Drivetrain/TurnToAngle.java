package frc.robot.commands.Drivetrain;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;

public class TurnToAngle extends PIDCommand {

    private final Drivetrain m_drivetrain;

    public TurnToAngle(Drivetrain drivetrain, double targetAngleDegrees) {
        super(
            new PIDController(
                Constants.Drivetrain.HEADING_PID[0],
                Constants.Drivetrain.HEADING_PID[1],
                Constants.Drivetrain.HEADING_PID[2]
            ),
            drivetrain::getHeading,
            targetAngleDegrees,
            output -> drivetrain.arcadeDrive(0, output),
            drivetrain
        );

        getController().enableContinuousInput(-180.0, 180.0);
        getController().setTolerance(Constants.Drivetrain.TURN_TOLERANCE);

        m_drivetrain = drivetrain;
    }

    @Override
    public void initialize() {
        m_drivetrain.resetGyro();
    }

    @Override
    public boolean isFinished() {
        return getController().atSetpoint();
    }

}