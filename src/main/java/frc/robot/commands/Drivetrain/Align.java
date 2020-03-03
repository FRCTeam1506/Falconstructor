package frc.robot.commands.Drivetrain;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;

public class Align extends PIDCommand {

    public Align(Drivetrain drivetrain) {
        super(
            new PIDController(
                Constants.Drivetrain.HEADING_PID[0], 
                Constants.Drivetrain.HEADING_PID[1], 
                Constants.Drivetrain.HEADING_PID[2]
            ),
            drivetrain::getX,
            2.0,
            output -> {
                System.out.println(-output);
                drivetrain.regArcadeDrive(0, -output);
            },
            drivetrain
        );

        getController().enableContinuousInput(-27.0, 27.0);
        getController().setTolerance(
            Constants.Drivetrain.TURN_TOLERANCE
        );
    }

    public Align(Drivetrain drivetrain, Double targetXError) {
        super(
            new PIDController(
                Constants.Drivetrain.HEADING_PID[0], 
                Constants.Drivetrain.HEADING_PID[1], 
                Constants.Drivetrain.HEADING_PID[2]
            ),
            drivetrain::getX,
            targetXError,
            output -> {
                System.out.println(-output);
                drivetrain.regArcadeDrive(0, -output);
            },
            drivetrain
        );

        getController().enableContinuousInput(-27.0, 27.0);
        getController().setTolerance(
            Constants.Drivetrain.TURN_TOLERANCE
        );
    }

    @Override
    public boolean isFinished() {
        return getController().atSetpoint();
    }

}