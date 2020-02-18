package frc.robot.subsystems;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Transform2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpiutil.math.MathUtil;

import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.commands.Drivetrain.DriveTrajectory;

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

    private DifferentialDriveOdometry odometry;
    private RamseteController ramseteController;
    private SimpleMotorFeedforward feedforward;

    private Timer pathTimer;

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

        // this.leftDriveMaster.setSensorPhase(true);
        // this.rightDriveMaster.setSensorPhase(true);

        this.leftDriveMaster.setInverted(false);
        this.rightDriveMaster.setInverted(true);
        this.leftDrive.setInverted(talonFXInvertType);
        this.rightDrive.setInverted(talonFXInvertType);

        this.leftDriveMaster.setNeutralMode(NeutralMode.Brake);
        this.rightDriveMaster.setNeutralMode(NeutralMode.Brake);

        this.leftDriveMaster.configSupplyCurrentLimit(supplyCurrentLimitConfiguration);
        this.rightDriveMaster.configSupplyCurrentLimit(supplyCurrentLimitConfiguration);

        feedforward = new SimpleMotorFeedforward(Constants.Drivetrain.kS, Constants.Drivetrain.kV, Constants.Drivetrain.kA);

        resetEncoders();
        odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getHeading()));
        ramseteController = new RamseteController(2.0, 0.7);

        pathTimer = new Timer();
    }

    public void fwd(double pwr) {
        this.leftDriveMaster.set(ControlMode.PercentOutput, pwr);
        this.rightDriveMaster.set(ControlMode.PercentOutput, pwr);
    }

    public void rot(double pwr) {
        this.leftDriveMaster.set(ControlMode.PercentOutput, pwr);
        this.rightDriveMaster.set(ControlMode.PercentOutput, -pwr);
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
        this.leftDriveMaster.set(ControlMode.PercentOutput, MathUtil.clamp(-leftVolts/12.0, -Constants.Drivetrain.MAX_VOLTS, Constants.Drivetrain.MAX_VOLTS));
        this.rightDriveMaster.set(ControlMode.PercentOutput, MathUtil.clamp(-rightVolts/12.0, -Constants.Drivetrain.MAX_VOLTS, Constants.Drivetrain.MAX_VOLTS));
    }

    public void limitedArcadeDrive(double fwd, double rot) {
        this.leftDriveMaster.set(ControlMode.PercentOutput, Math.pow((fwd + rot) * 0.85, 3));
        this.rightDriveMaster.set(ControlMode.PercentOutput, Math.pow((fwd - rot) * 0.85, 3));
    }

    public void arcadeDrive(double fwd, double rot) {
        this.leftDriveMaster.set(ControlMode.PercentOutput, Math.pow((fwd + rot), 3));
        this.rightDriveMaster.set(ControlMode.PercentOutput, Math.pow((fwd - rot), 3));
    }

    public void regArcadeDrive(double fwd, double rot) {
        this.leftDriveMaster.set(ControlMode.PercentOutput, (fwd + rot));
        this.rightDriveMaster.set(ControlMode.PercentOutput, (fwd - rot));
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
        return Math.IEEEremainder(navx.getAngle(), 360);
    }

    public Pose2d getPose() {
        return odometry.getPoseMeters();
    }

    public double getTurnRate() {
        return navx.getRate();
    }

    public double getLeftDistanceMeters() {
        return -leftDriveMaster.getSelectedSensorPosition() / Constants.Drivetrain.LEFT_TICKS_PER_REV;
    }

    public double getRightDistanceMeters() {
        return -rightDriveMaster.getSelectedSensorPosition() / Constants.Drivetrain.RIGHT_TICKS_PER_REV;
    }

    public double getLeftVelocityMetersPerSecond() {
        return -leftDriveMaster.getSelectedSensorVelocity() * (10.0 / Constants.Drivetrain.LEFT_TICKS_PER_REV);
    }

    public double getRightVelocityMetersPerSecond() {
        return -rightDriveMaster.getSelectedSensorVelocity() * (10.0 / Constants.Drivetrain.RIGHT_TICKS_PER_REV);
    }

    public double getAverageEncoderDistanceMeters() {
        return ( ( getLeftDistanceMeters() + getRightDistanceMeters() ) / 2.0 );
    }

    public DifferentialDriveWheelSpeeds getWheelSpeeds() {
        return new DifferentialDriveWheelSpeeds(
            getLeftVelocityMetersPerSecond(), 
            getRightVelocityMetersPerSecond()
        );
    }

    public void resetGyro() {
    	navx.zeroYaw();
    }

    public void resetOdometry(Pose2d pose) {
        resetEncoders();
        odometry.resetPosition(pose, Rotation2d.fromDegrees(getHeading()));
    }

    public void resetAll() {
        resetEncoders();
        resetGyro();
        // resetOdometry(new Pose2d(3.195,-2.32, new Rotation2d(0.0)));
    }

    public void driveTrajectory(Trajectory trajectory, boolean reversedTrajectory) {
        pathTimer.reset();
        pathTimer.start();
        Trajectory.State currentStateTraj = trajectory.sample(pathTimer.get());
        ChassisSpeeds ramseteChassisSpeeds;

        if (!reversedTrajectory) {
            ramseteChassisSpeeds = ramseteController.calculate(
                odometry.getPoseMeters(), 
                currentStateTraj);
        }
        else {
            ramseteChassisSpeeds = ramseteController.calculate(
                odometry.getPoseMeters().transformBy(new Transform2d(new Translation2d(0, 0), new Rotation2d(Math.PI))), 
                currentStateTraj);
        }
        DifferentialDriveWheelSpeeds ramseteDSpeeds = Constants.Drivetrain.kDriveKinematics.toWheelSpeeds(ramseteChassisSpeeds);

        double leftSetpoint;
        double rightSetpoint;

        if (!reversedTrajectory) {
            leftSetpoint = ramseteDSpeeds.leftMetersPerSecond;
            rightSetpoint = ramseteDSpeeds.rightMetersPerSecond;
        }
        else {
            leftSetpoint = -ramseteDSpeeds.rightMetersPerSecond;
            rightSetpoint = -ramseteDSpeeds.leftMetersPerSecond;
        }

        // use P to acquire desired velocity and feedforward to acquire desired velocity
        double ramseteLeftError = leftSetpoint - leftDriveMaster.getSelectedSensorVelocity();
        double ramseteRightError = rightSetpoint - rightDriveMaster.getSelectedSensorVelocity();

        double leftVolts = ramseteLeftError * 0.01 + feedforward.calculate(leftSetpoint);
        double rightVolts = ramseteRightError * 0.01 + feedforward.calculate(rightSetpoint);

        System.out.println("Left " + leftVolts);
        System.out.println("Right " + rightVolts);

        this.tankDriveVolts(
            rightVolts,
            leftVolts
        );

        // this.tankDrive(leftVolts, rightVolts);

        if (trajectory.getTotalTimeSeconds() <= pathTimer.get()) {
            pathTimer.stop();
            trajectory = null;
        }
    }

    @Override
    public void periodic() {
        odometry.update(
            Rotation2d.fromDegrees(getHeading()), 
            getLeftDistanceMeters(), 
            getRightDistanceMeters()
        );
        // if(!(CommandScheduler.getInstance().isScheduled(RobotContainer.getGyroAlign()))) {
        //     this.targetHeading = getHeading();
        // }
        // this.targetHeading = getHeading();
        SmartDashboard.putNumber("[Drivetrain]-Target-Heading", getTargetHeading());
        SmartDashboard.putNumber("[Drivetrain]-Heading", getHeading());
        SmartDashboard.putNumber("[Drivetrain]-Left-Dist-Meters", getLeftDistanceMeters());
        SmartDashboard.putNumber("[Drivetrain]-Right-Dist-Meters", getRightDistanceMeters());
        SmartDashboard.putNumber("[Drivetrain]-Left-Ticks", -this.leftDriveMaster.getSelectedSensorPosition());
        SmartDashboard.putNumber("[Drivetrain]-Right-Ticks", -this.rightDriveMaster.getSelectedSensorPosition());
    }
}