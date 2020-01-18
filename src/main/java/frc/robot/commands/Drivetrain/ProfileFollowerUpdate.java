package frc.robot.commands.Drivetrain;

import java.io.File;
import java.io.IOException;

import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.followers.EncoderFollower;

public class ProfileFollowerUpdate extends CommandBase {
    private final TalonFX leftMotor;
    private final TalonFX rightMotor;
    private EncoderFollower left;
    private EncoderFollower right;
    private final Trajectory leftTra;
    private final Trajectory rightTra;

    public ProfileFollowerUpdate(String leftCSV, String rightCSV) throws IOException {
        addRequirements(RobotContainer.drivetrain);
        File leftMotionProfile = new File(leftCSV);
        File rightMotionProfile = new File(rightCSV);

        leftMotor = RobotContainer.drivetrain.leftDriveMaster;
        rightMotor = RobotContainer.drivetrain.rightDriveMaster;
        leftTra = Pathfinder.readFromCSV(leftMotionProfile);
        rightTra = Pathfinder.readFromCSV(rightMotionProfile);
    }

    /**
     * The initialize method is called the first time this Command is run after being started.
     */
    @Override
    public void initialize() {
        // super.initialize();
        RobotContainer.drivetrain.navx.zeroYaw();
        RobotContainer.drivetrain.resetEncoders();
        left = new EncoderFollower(leftTra);
        right = new EncoderFollower(rightTra);

        left.configureEncoder(leftMotor.getSelectedSensorPosition(0), 30000, 0.5);
        right.configureEncoder(rightMotor.getSelectedSensorPosition(0), 30000, 0.5);

        double max_velocity = 1.0 / 4.0;
        left.configurePIDVA(0.4, 0.0, 0.07, max_velocity, 0);
        right.configurePIDVA(0.4, 0.0, 0.07, max_velocity, 0);
    }

    /**
     * Called when the command ended peacefully. This is where you may want to wrap up loose ends,
     * like shutting off a motor that was being used in the command.
     */
    @Override
    public void end(boolean interrupted) {
        // super.end(interrupted);
        RobotContainer.drivetrain.drive(0, 0);
    }

    /**
     * The execute method is called repeatedly until this Command either finishes or is canceled.
     */
    @Override
    public void execute() {
        // super.execute();
        System.err.println("Execute ProfileFollower.");
        double l = left.calculate(leftMotor.getSelectedSensorPosition(0));
        double r = right.calculate(rightMotor.getSelectedSensorPosition(0));
        double gyro_heading = RobotContainer.drivetrain.navx.getAngle();
        double desired_heading = Pathfinder.r2d(left.getHeading());
        double angleDifference = Pathfinder.boundHalfDegrees(desired_heading - gyro_heading);
        System.out.println("Desired angle  " + desired_heading + "Current heading " + gyro_heading + "Angle difference " + angleDifference);
        double turn = 1.2  * (-1.0/80.0) * angleDifference;
        System.out.println("Left: " + (l));
        System.out.println("Right: " + (r));
        System.out.println("Left + turn: " + (l+turn));
        System.out.println("Right + turn: " + (r-turn));
        RobotContainer.drivetrain.drive(l+turn, r-turn);
    }
}