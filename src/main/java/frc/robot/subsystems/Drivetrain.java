package frc.robot.subsystems;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;

import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.Waypoint;
import jaci.pathfinder.followers.EncoderFollower;
import jaci.pathfinder.modifiers.TankModifier;

import java.io.File;
import java.io.IOException;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.kauailabs.navx.frc.AHRS;

public class Drivetrain extends SubsystemBase {

    public TalonFX leftDriveMaster = new TalonFX(Constants.Drivetrain.LEFT_DRIVE_MASTER_ID.getID());
    public TalonFX leftDrive = new TalonFX(Constants.Drivetrain.LEFT_DRIVE_ID.getID());
    public TalonFX rightDriveMaster = new TalonFX(Constants.Drivetrain.RIGHT_DRIVE_MASTER_ID.getID());
    public TalonFX rightDrive = new TalonFX(Constants.Drivetrain.RIGHT_DRIVE_ID.getID());

    private TalonFXInvertType talonFXInvertType = TalonFXInvertType.FollowMaster;

    public AHRS navx = new AHRS(SPI.Port.kMXP);

    public boolean isProfileFinished;

    public Drivetrain() {
        this.leftDrive.configFactoryDefault();
        this.leftDriveMaster.configFactoryDefault();
        this.rightDrive.configFactoryDefault();
        this.rightDriveMaster.configFactoryDefault();

        // Set followers
        this.leftDrive.follow(this.leftDriveMaster);
        this.rightDrive.follow(this.rightDriveMaster);

        this.leftDriveMaster.setInverted(true);
        this.leftDrive.setInverted(talonFXInvertType);
        this.rightDrive.setInverted(talonFXInvertType);

        this.leftDriveMaster.setNeutralMode(NeutralMode.Coast);
        this.rightDriveMaster.setNeutralMode(NeutralMode.Coast);

        resetEncoders();
    }

    public void drive(double leftPower, double rightPower) {
        this.leftDriveMaster.set(ControlMode.PercentOutput, leftPower);
        this.rightDriveMaster.set(ControlMode.PercentOutput, rightPower);
    }

    public void tankDrive(double leftSpeed, double rightSpeed) {
        this.leftDriveMaster.set(ControlMode.PercentOutput, leftSpeed);
        this.rightDriveMaster.set(ControlMode.PercentOutput, rightSpeed);
    }

    public void limitedArcadeDrive(double fwd, double rot) {
        this.leftDriveMaster.set(ControlMode.PercentOutput, (fwd + rot) * 0.65);
        this.rightDriveMaster.set(ControlMode.PercentOutput, (fwd - rot) * 0.65);
    }

    public void arcadeDrive(double fwd, double rot) {
        this.leftDriveMaster.set(ControlMode.PercentOutput, fwd + rot);
        this.rightDriveMaster.set(ControlMode.PercentOutput, fwd - rot);
    }

    public void stop() {
        this.leftDriveMaster.set(ControlMode.PercentOutput, 0.0);
        this.rightDriveMaster.set(ControlMode.PercentOutput, 0.0);
    }

    public void resetEncoders() {
        this.leftDriveMaster.setSelectedSensorPosition(0, 0, 0);
        this.rightDriveMaster.setSelectedSensorPosition(0, 0, 0);
    }

    public void resetGyro() {
    	navx.zeroYaw();
    }

    public void resetForPath() {
        isProfileFinished = false;
        resetEncoders();
        resetGyro();
    }

    public double generateHashCode(Waypoint[] path) {
        double hash = 1.0;
        for (int i = 0; i < path.length; i++) {
            hash = ((path[i].x * 6) + (path[i].y * 3) + (path[i].angle * 25));
        }
        return (int) Math.abs(hash * 100);
    }

    public EncoderFollower[] initPath(String leftCSV, String rightCSV) throws IOException {
        resetEncoders();
        File leftMotionProfile = new File(leftCSV);
        File rightMotionProfile = new File(rightCSV);
        System.err.println("File about to read");
        Trajectory leftTrajectory = Pathfinder.readFromCSV(leftMotionProfile);
        Trajectory rightTrajectory = Pathfinder.readFromCSV(rightMotionProfile);
        System.err.println("File read");
        EncoderFollower left = new EncoderFollower(leftTrajectory);
        EncoderFollower right = new EncoderFollower(rightTrajectory);
        left.configureEncoder(leftDriveMaster.getSelectedSensorPosition(0), MotionProfiling.ticks_per_rev,
                MotionProfiling.wheel_diameter);
        right.configureEncoder(rightDriveMaster.getSelectedSensorPosition(0), MotionProfiling.ticks_per_rev,
                MotionProfiling.wheel_diameter);
        left.configurePIDVA(MotionProfiling.kp, MotionProfiling.ki, MotionProfiling.kd, MotionProfiling.kv,
                MotionProfiling.ka);
        right.configurePIDVA(MotionProfiling.kp, MotionProfiling.ki, MotionProfiling.kd, MotionProfiling.kv,
                MotionProfiling.ka);
        navx.zeroYaw();
        return new EncoderFollower[] {
            left, // 0 left
            right, // 1 right
        };
    }

    public EncoderFollower[] initPath(Waypoint[] path) throws IOException {

        EncoderFollower left = new EncoderFollower();
        EncoderFollower right = new EncoderFollower();
        Trajectory.Config cfg = new Trajectory.Config(Trajectory.FitMethod.HERMITE_QUINTIC, Trajectory.Config.SAMPLES_HIGH,
                Drivetrain.MotionProfiling.dt, Drivetrain.MotionProfiling.max_velocity, Drivetrain.MotionProfiling.max_acceleration, Drivetrain.MotionProfiling.max_jerk);
        String pathHash = String.valueOf(generateHashCode(path));
        SmartDashboard.putString("Path Hash", pathHash);
        Trajectory toFollow;
        File trajectory = new File("/home/lvuser/paths/" + pathHash + ".csv");
        if (!trajectory.exists()) {
            toFollow = Pathfinder.generate(path, cfg);
            Pathfinder.writeToCSV(trajectory, toFollow);
            System.out.println(pathHash + ".csv not found, wrote to file");
        } else {
            System.out.println(pathHash + ".csv read from file");
            toFollow = Pathfinder.readFromCSV(trajectory);
        }

        TankModifier modifier = new TankModifier(toFollow).modify((Drivetrain.MotionProfiling.wheel_base_width));
        left = new EncoderFollower(modifier.getLeftTrajectory());
        right = new EncoderFollower(modifier.getRightTrajectory());
        left.configureEncoder(leftDriveMaster.getSelectedSensorPosition(0), MotionProfiling.ticks_per_rev, MotionProfiling.wheel_diameter);
        right.configureEncoder(rightDriveMaster.getSelectedSensorPosition(0), MotionProfiling.ticks_per_rev, MotionProfiling.wheel_diameter);
        left.configurePIDVA(MotionProfiling.kp, MotionProfiling.ki, MotionProfiling.kd, MotionProfiling.kv, MotionProfiling.ka);
        right.configurePIDVA(MotionProfiling.kp, MotionProfiling.ki, MotionProfiling.kd, MotionProfiling.kv, MotionProfiling.ka);
        return new EncoderFollower[]{
            left, // 0
            right, // 1
        };
    }
    public void executePath(EncoderFollower[] followers, boolean reverse) {
    	EncoderFollower left = followers[0];
        EncoderFollower right = followers[1];
        double l,r;
        if (!reverse) {
            l = left.calculate(leftDriveMaster.getSelectedSensorPosition(0));
            r = right.calculate(rightDriveMaster.getSelectedSensorPosition(0));
        } else {
            l = left.calculate(-leftDriveMaster.getSelectedSensorPosition(0));
            r = right.calculate(-rightDriveMaster.getSelectedSensorPosition(0));
        }
        double gyro_heading = navx.getYaw();
        double angle_setpoint = Pathfinder.r2d(left.getHeading());
        double angleDifference = Pathfinder.boundHalfDegrees(angle_setpoint - gyro_heading);
        double turn = 0.8 * (-1.0/80.0) * angleDifference;
        if(!reverse) {
            drive(l + turn, r - turn);
        }
        else {
            drive(-l + turn, -r - turn);
        }
        if(left.isFinished() && right.isFinished()) {
        	isProfileFinished = true;
        }
    }

    public boolean isProfileFinished() {
        return isProfileFinished;
    }

    public static class MotionProfiling {
        
        public static double kp = 0.025;   // 1 -> 0.5
        public static double ki = 0.0; // not used
        public static double kd = 0.05;

        public static final double max_velocity = 0.7;      // 7.0
        // public static final double kv = 1 / max_velocity;
        public static final double kv = 1 / 7;
        public static final double max_acceleration = 1.0;  // 3.0
        public static final double max_jerk = 0.0; // 60.0
        public static final double ka = 0.0;                // to get to higher or lower speed quicker
        public static final double wheel_diameter = 0.5;
        public static final double wheel_base_width = 27.11/12.0;
        public static final double wheel_circumference = 4*Math.PI;
        public static final int ticks_per_rev = 13000; // CTRE Mag Encoder // 4096*7
        public static final double distancePerPulse = (wheel_diameter*Math.PI)/ticks_per_rev;
        public static final double dt = 0.05;
	}
}