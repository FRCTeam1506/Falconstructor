package frc.robot.commands.Drivetrain;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;

public class DriveBackward extends PIDCommand {

    public DriveBackward(Drivetrain drivetrain, Double pwr) {
        super(
            new PIDController(
                Constants.Drivetrain.STABILIZATION_PID[0],
                Constants.Drivetrain.STABILIZATION_PID[1],
                Constants.Drivetrain.STABILIZATION_PID[2]
            ),
            drivetrain::getTurnRate,
            0.0,
            output -> drivetrain.arcadeDrive(pwr, output),
            drivetrain
        );
    }

}