package frc.robot.commands.Drivetrain;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;

public class FastTurnToAngleProfiled extends ProfiledPIDCommand {

    private final Drivetrain m_drivetrain;

    public FastTurnToAngleProfiled(Drivetrain drivetrain, double targetAngleDegrees) {
        super(
            new ProfiledPIDController(
                Constants.Drivetrain.FAST_HEADING_PID[0],
                Constants.Drivetrain.FAST_HEADING_PID[1],
                Constants.Drivetrain.FAST_HEADING_PID[2],
                new TrapezoidProfile.Constraints(Constants.Drivetrain.MAX_TURN_RATE, Constants.Drivetrain.MAX_TURN_ACCEL)
            ),
            drivetrain::getHeading,
            targetAngleDegrees,
            (output, setpoint) -> drivetrain.arcadeDrive(0, -output),
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
        return getController().atGoal();
    }

}