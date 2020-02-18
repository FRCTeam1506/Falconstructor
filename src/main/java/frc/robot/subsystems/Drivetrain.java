package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.RobotContainer;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
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

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.kauailabs.navx.frc.AHRS;

public class Drivetrain extends SubsystemBase {

    //? Drives
    private final TalonFX leftDriveMaster = new TalonFX(Constants.Drivetrain.LEFT_DRIVE_MASTER_ID.getID());
    private final TalonFX leftDrive = new TalonFX(Constants.Drivetrain.LEFT_DRIVE_ID.getID());
    private final TalonFX rightDriveMaster = new TalonFX(Constants.Drivetrain.RIGHT_DRIVE_MASTER_ID.getID());
    private final TalonFX rightDrive = new TalonFX(Constants.Drivetrain.RIGHT_DRIVE_ID.getID());

    private TalonFXInvertType talonFXInvertType = TalonFXInvertType.FollowMaster;

    private SupplyCurrentLimitConfiguration supplyCurrentLimitConfiguration = new SupplyCurrentLimitConfiguration(true, 100.0, 120.0, 0);

    //? Limelight
    private boolean aligned, isRefreshed, targetFound;
    private Double x, y, area, targetDistance, previousX, previousY, previousDist;
    private Integer pipeline;

    private NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");

    public enum Piplelines {
        NearTargeting,
        FarTargeting,
        Red,
        Green,
        Blue,
        Yellow
    }

    //? Gyro
    private final AHRS navx = new AHRS(SPI.Port.kMXP);

    //? Trajectory
    private DifferentialDriveOdometry odometry;
    private RamseteController ramseteController;
    private SimpleMotorFeedforward feedforward;

    public Drivetrain() {
        this.leftDrive.configFactoryDefault();
        this.leftDriveMaster.configFactoryDefault();
        this.rightDrive.configFactoryDefault();
        this.rightDriveMaster.configFactoryDefault();

        // Set followers
        this.leftDrive.follow(this.leftDriveMaster);
        this.rightDrive.follow(this.rightDriveMaster);

        this.leftDriveMaster.setInverted(false);
        this.rightDriveMaster.setInverted(true);
        this.leftDrive.setInverted(talonFXInvertType);
        this.rightDrive.setInverted(talonFXInvertType);

        this.leftDriveMaster.setNeutralMode(NeutralMode.Brake);
        this.rightDriveMaster.setNeutralMode(NeutralMode.Brake);

        this.leftDriveMaster.configSupplyCurrentLimit(supplyCurrentLimitConfiguration);
        this.rightDriveMaster.configSupplyCurrentLimit(supplyCurrentLimitConfiguration);

        this.targetDistance = 5000.0;

        resetEncoders();

        this.feedforward = new SimpleMotorFeedforward(Constants.Drivetrain.kS, Constants.Drivetrain.kV, Constants.Drivetrain.kA);
        this.odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getHeading()));
        this.ramseteController = new RamseteController(2.0, 0.7);
    }

    //? Drive
    public void fwd(double pwr) {
        this.leftDriveMaster.set(ControlMode.PercentOutput, pwr);
        this.rightDriveMaster.set(ControlMode.PercentOutput, pwr);
    }

    public void rot(double pwr) {
        this.leftDriveMaster.set(ControlMode.PercentOutput, pwr);
        this.rightDriveMaster.set(ControlMode.PercentOutput, -pwr);
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

    //? Limelight
    public void setPipeline(int index) {
        this.table.getEntry("pipeline").setNumber(index);
    }

    public void setPipeline(Piplelines pipe) {
        switch (pipe) {
            case NearTargeting:
                this.setPipeline(5);
                break;

            case FarTargeting:
                this.setPipeline(6);
                break;

            case Red:
                this.setPipeline(0);
                break;

            case Green:
                this.setPipeline(1);
                break;

            case Blue:
                this.setPipeline(2);
                break;

            case Yellow:
                this.setPipeline(3);
                break;
        
            default:
                this.setPipeline(5);
                break;
        }
    }

    public void setTargetDistance(Double distance) {
        this.targetDistance = distance;
    }

    public Double getX() {
        return this.x;
    }

    public Double getY() {
        return this.y;
    }

    public Double getArea() {
        return this.area;
    }

    public int getPipeline() {
        return this.pipeline;
    }

    public boolean isRefreshed() {
        return this.isRefreshed;
    }

    public Double getDistance() {
        double val;
        // inches
        Double h2 = 96.0;
        Double h1 = 30.0;
        Double a1 = 1.0; // 0.258
        Double a2 = this.y;
        // return (h2 - h1) / Math.tan(a1 + a2);
        double dist = (h2 - h1) / (Math.tan((a1 + a2) * (Math.PI / 180)));
        if(dist > 0) {
            val = (double) Math.ceil((dist / 1000.0)) * 1000.0;
            if(Math.abs(val) < 1) this.previousDist = val;
        } else {
            val = this.previousDist;
        }
        return val;
    }

    public Double getTargetDistance() {
        return this.targetDistance;
    }

    public boolean isTargetFound() {
        return this.targetFound;
    }

    public boolean isAligned() {
        return this.aligned;
    }

    //? Gyro
    public void resetGyro() {
    	navx.zeroYaw();
    }

    //? Trajectory
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
        return leftDriveMaster.getSelectedSensorPosition() / Constants.Drivetrain.LEFT_TICKS_PER_REV;
    }

    public double getRightDistanceMeters() {
        return rightDriveMaster.getSelectedSensorPosition() / Constants.Drivetrain.RIGHT_TICKS_PER_REV;
    }

    public double getLeftVelocityMetersPerSecond() {
        return leftDriveMaster.getSelectedSensorVelocity() * (10.0 / Constants.Drivetrain.LEFT_TICKS_PER_REV);
    }

    public double getRightVelocityMetersPerSecond() {
        return rightDriveMaster.getSelectedSensorVelocity() * (10.0 / Constants.Drivetrain.RIGHT_TICKS_PER_REV);
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

    public void resetOdometry(Pose2d pose) {
        resetEncoders();
        odometry.resetPosition(pose, Rotation2d.fromDegrees(getHeading()));
    }

    public void resetAll() {
        resetEncoders();
        resetGyro();
    }

    @Override
    public void periodic() {
        odometry.update(
            Rotation2d.fromDegrees(getHeading()), 
            getLeftDistanceMeters(), 
            getRightDistanceMeters()
        );

        this.x = table.getEntry("tx").getDouble(0.0);
        this.y = table.getEntry("ty").getDouble(0.0);
        this.area = table.getEntry("ta").getDouble(0.0);
        this.targetFound = table.getEntry("tv").getNumber(0).intValue() == 1 ? true : false;
        this.pipeline = table.getEntry("pipeline").getNumber(0).intValue();
        this.aligned = Math.abs(x) < 0.1 ? true : false;
        this.isRefreshed = this.x != previousX || this.y != previousY ? true : false;
        this.previousX = this.x;
        this.previousY = this.y;

        SmartDashboard.putNumber("[Limelight]-Target-Distance", getTargetDistance());
        SmartDashboard.putNumber("[Limelight]-Distance", getDistance());

        SmartDashboard.putNumber("[Drivetrain]-Heading", getHeading());
        SmartDashboard.putNumber("[Drivetrain]-Left-Dist-Meters", getLeftDistanceMeters());
        SmartDashboard.putNumber("[Drivetrain]-Right-Dist-Meters", getRightDistanceMeters());
        SmartDashboard.putNumber("[Drivetrain]-Left-Ticks", this.leftDriveMaster.getSelectedSensorPosition());
        SmartDashboard.putNumber("[Drivetrain]-Right-Ticks", this.rightDriveMaster.getSelectedSensorPosition());
    }
}