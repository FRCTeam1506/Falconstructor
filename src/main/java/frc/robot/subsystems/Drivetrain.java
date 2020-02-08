package frc.robot.subsystems;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpiutil.math.MathUtil;

import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.commands.Drivetrain.ArcadeDrive;
import frc.robot.commands.Drivetrain.FastArcadeDrive;
import frc.robot.commands.Drivetrain.LimitedArcadeDrive;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.kauailabs.navx.frc.AHRS;

public class Drivetrain extends SubsystemBase {

    public TalonFX leftDriveMaster = new TalonFX(Constants.Drivetrain.LEFT_DRIVE_MASTER_ID.getID());
    public TalonFX leftDrive = new TalonFX(Constants.Drivetrain.LEFT_DRIVE_ID.getID());
    public TalonFX rightDriveMaster = new TalonFX(Constants.Drivetrain.RIGHT_DRIVE_MASTER_ID.getID());
    public TalonFX rightDrive = new TalonFX(Constants.Drivetrain.RIGHT_DRIVE_ID.getID());

    private TalonFXInvertType talonFXInvertType = TalonFXInvertType.FollowMaster;

    private SupplyCurrentLimitConfiguration supplyCurrentLimitConfiguration = new SupplyCurrentLimitConfiguration(true, 100.0, 120.0, 0);

    public AHRS navx = new AHRS(SPI.Port.kMXP);

    public DifferentialDriveOdometry odometry;

    public boolean isProfileFinished;

    private double targetHeading;

    public Drivetrain() {
        this.leftDrive.configFactoryDefault();
        this.leftDriveMaster.configFactoryDefault();
        this.rightDrive.configFactoryDefault();
        this.rightDriveMaster.configFactoryDefault();

        // Set followers
        this.leftDrive.follow(this.leftDriveMaster);
        this.rightDrive.follow(this.rightDriveMaster);

        this.leftDriveMaster.setSensorPhase(false);
        this.rightDriveMaster.setSensorPhase(false);

        this.leftDriveMaster.setInverted(false);
        this.rightDriveMaster.setInverted(true);
        this.leftDrive.setInverted(talonFXInvertType);
        this.rightDrive.setInverted(talonFXInvertType);

        this.leftDriveMaster.setNeutralMode(NeutralMode.Brake);
        this.rightDriveMaster.setNeutralMode(NeutralMode.Brake);

        this.leftDriveMaster.configSupplyCurrentLimit(supplyCurrentLimitConfiguration);
        this.rightDriveMaster.configSupplyCurrentLimit(supplyCurrentLimitConfiguration);

        resetEncoders();
        odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getHeading()));
    }

    @Override
    public void periodic() {
        odometry.update(
            Rotation2d.fromDegrees(getHeading()), 
            getLeftDistanceMeters(), 
            getRightDistanceMeters()
        );
    }

    public void drive(double leftPower, double rightPower) {
        this.leftDriveMaster.set(ControlMode.PercentOutput, leftPower);
        this.rightDriveMaster.set(ControlMode.PercentOutput, rightPower);
    }

    public void tankDrive(double leftSpeed, double rightSpeed) {
        this.leftDriveMaster.set(ControlMode.PercentOutput, leftSpeed);
        this.rightDriveMaster.set(ControlMode.PercentOutput, rightSpeed);
    }

    public void tankDriveVolts(double leftVolts, double rightVolts) {
        this.leftDriveMaster.set(ControlMode.PercentOutput, MathUtil.clamp(leftVolts/12, -Constants.Drivetrain.MAX_VOLTS, Constants.Drivetrain.MAX_VOLTS));
        this.rightDriveMaster.set(ControlMode.PercentOutput, MathUtil.clamp(rightVolts/12, -Constants.Drivetrain.MAX_VOLTS, Constants.Drivetrain.MAX_VOLTS));
    }

    public void limitedArcadeDrive(double fwd, double rot) {
        this.leftDriveMaster.set(ControlMode.PercentOutput, Math.pow((fwd + rot) * 0.65, 3));
        this.rightDriveMaster.set(ControlMode.PercentOutput, Math.pow((fwd - rot) * 0.65, 3));
    }

    public void arcadeDrive(double fwd, double rot) {
        this.leftDriveMaster.set(ControlMode.PercentOutput, Math.pow((fwd + rot + 0.05), 3));
        this.rightDriveMaster.set(ControlMode.PercentOutput, Math.pow((fwd - rot), 3));
    }

    public void stop() {
        this.leftDriveMaster.set(ControlMode.PercentOutput, 0.0);
        this.rightDriveMaster.set(ControlMode.PercentOutput, 0.0);
    }

    public void resetEncoders() {
        this.leftDriveMaster.setSelectedSensorPosition(0, 0, 0);
        this.rightDriveMaster.setSelectedSensorPosition(0, 0, 0);
    }

    public double getTargetHeading() {
        return this.targetHeading;
    }

    public void setTargetHeading(double heading) {
        this.targetHeading = heading;
    }

    public Double getHeading() {
        return -Math.IEEEremainder(navx.getAngle(), 360);
    }

    public Pose2d getPose() {
        return odometry.getPoseMeters();
    }

    public double getTurnRate() {
        return navx.getRate();
    }

    public DifferentialDriveWheelSpeeds getWheelSpeeds() {
        return new DifferentialDriveWheelSpeeds((leftDriveMaster.getSelectedSensorVelocity() * Constants.Drivetrain.GEAR_RATIO * 10 / Constants.General.LEFT_TICKS_PER_REV) * Constants.Drivetrain.WHEEL_CIRCUMFERENCE_METERS, 
            (rightDriveMaster.getSelectedSensorVelocity() * Constants.Drivetrain.GEAR_RATIO * 10 / Constants.General.RIGHT_TICKS_PER_REV) * Constants.Drivetrain.WHEEL_CIRCUMFERENCE_METERS);
    }

    public double getLeftDistanceMeters() {
        return (leftDriveMaster.getSelectedSensorPosition() / Constants.General.LEFT_TICKS_PER_REV * Constants.Drivetrain.GEAR_RATIO) * Constants.Drivetrain.WHEEL_CIRCUMFERENCE_METERS;
    }

    public double getRightDistanceMeters() {
        return (rightDriveMaster.getSelectedSensorPosition() / Constants.General.RIGHT_TICKS_PER_REV * Constants.Drivetrain.GEAR_RATIO) * Constants.Drivetrain.WHEEL_CIRCUMFERENCE_METERS;
    }

    public double getAverageEncoderDistance() {
        return ( ( getLeftDistanceMeters() + getRightDistanceMeters() ) / 2.0);
    }

    public void resetGyro() {
    	navx.zeroYaw();
    }

    public void resetOdometry(Pose2d pose) {
        resetEncoders();
        odometry.resetPosition(pose, Rotation2d.fromDegrees(getHeading()));
    }
}