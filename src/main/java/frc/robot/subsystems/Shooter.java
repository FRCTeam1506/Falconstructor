package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {

    // Instantiate Hardware
    public TalonFX shooter1 = new TalonFX(Constants.Shooter.SHOOTER_1_ID.getID());
    public TalonFX shooter2 = new TalonFX(Constants.Shooter.SHOOTER_2_ID.getID());

    // public TalonFX shooter1 = null;
    // public TalonFX shooter2 = null;

    private double targetVelocity;

    public Shooter() {
        this.shooter1.configFactoryDefault();
        this.shooter2.configFactoryDefault();

        this.shooter1.setInverted(true);
        this.shooter2.setInverted(false);

        this.shooter1.setNeutralMode(NeutralMode.Brake);
        this.shooter2.setNeutralMode(NeutralMode.Brake);

        this.shooter1.configNominalOutputForward(0);
        this.shooter1.configNominalOutputReverse(0);
        this.shooter1.configPeakOutputForward(1);
        this.shooter1.configPeakOutputReverse(-1);

        this.shooter2.configNominalOutputForward(0);
        this.shooter2.configNominalOutputReverse(0);
        this.shooter2.configPeakOutputForward(1);
        this.shooter2.configPeakOutputReverse(-1);

        this.shooter1.config_kP(0, Constants.Shooter.kP);
        this.shooter1.config_kI(0, Constants.Shooter.kI);
        this.shooter1.config_kD(0, Constants.Shooter.kD);
        // this.shooter1.config_kF(0, 1023.0/7777.0);
        // this.shooter1.config_kF(0, 0.2);
        this.shooter1.config_kF(0, Constants.Shooter.kF);

        this.shooter2.config_kP(0, Constants.Shooter.kP);
        this.shooter2.config_kI(0, Constants.Shooter.kI);
        this.shooter2.config_kD(0, Constants.Shooter.kD);
        // this.shooter2.config_kF(0, 1023.0/7777.0);
        // this.shooter2.config_kF(0, 0.2);
        this.shooter2.config_kF(0, Constants.Shooter.kF); // 0.044
    }

    private void setPower(Double pwr) {
        this.shooter1.set(ControlMode.PercentOutput, pwr);
        this.shooter2.set(ControlMode.PercentOutput, pwr);
    }

    private void setVelocity(Double velocity) {
        this.shooter1.set(ControlMode.Velocity, velocity); // -21600 -19800
        this.shooter2.set(ControlMode.Velocity, velocity);
    }

    // Subsystem Functions
    public void shoot(Double pwr) {
        this.setPower(pwr);
    }

    public void fullSend() {
        // this.shooter1.set(ControlMode.PercentOutput, -0.85);
        // this.shooter2.set(ControlMode.PercentOutput, -0.85);
        // this.shooter1.set(ControlMode.Velocity, -22200); // -21600 -19800
        // this.shooter2.set(ControlMode.Velocity, -22200);
        if(targetVelocity > -22200.0) {
            targetVelocity += -1000.0;
        } else {
            targetVelocity = -22200.0;
        }
        this.setVelocity(targetVelocity);
    }

    public void fullSend2() {
        // this.shooter1.set(ControlMode.PercentOutput, -0.85);
        // this.shooter2.set(ControlMode.PercentOutput, -0.85);
        // this.shooter1.set(ControlMode.Velocity, -38420);
        // this.shooter2.set(ControlMode.Velocity, -38420);
        this.setVelocity(-33333.0);
    }

    public void shootBasedOnDistance(Double distance) {
        // todo
        Double vel = distance;
    }

    public void stopShooter() {
        this.setPower(0.0);
    }

    public void resetTargetVelocity() {
        this.targetVelocity = 0.0;
    }
}