package frc.robot.commands.Drivetrain;

import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;

public class TurnToAngleProfiled extends ProfiledPIDCommand {

    public TurnToAngleProfiled(Drivetrain drivetrain, double targetAngleDegrees) {
        super(
            new ProfiledPIDController(
                Constants.Drivetrain.HEADING_PID[0], 
                Constants.Drivetrain.HEADING_PID[1], 
                Constants.Drivetrain.HEADING_PID[2], 
                new TrapezoidProfile.Constraints(Constants.Drivetrain.MAX_TURN_RATE, Constants.Drivetrain.MAX_TURN_ACCEL)
            ),
            drivetrain::getHeading,
            targetAngleDegrees,
            (output, setpoint) -> drivetrain.regArcadeDrive(0.0, output),
            drivetrain
        );

        getController().enableContinuousInput(-180.0, 180.0);
        getController().setTolerance(Constants.Drivetrain.TURN_TOLERANCE, Constants.Drivetrain.TURN_RATE_TOLERANCE);
    }

    @Override
    public boolean isFinished() {
        return getController().atGoal();
    }

}