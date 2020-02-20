package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {

    public TalonFX shooter1 = new TalonFX(Constants.Shooter.SHOOTER_1_ID.getID());
    public TalonFX shooter2 = new TalonFX(Constants.Shooter.SHOOTER_2_ID.getID());

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
        this.shooter1.config_kF(0, Constants.Shooter.kF);

        this.shooter2.config_kP(0, Constants.Shooter.kP);
        this.shooter2.config_kI(0, Constants.Shooter.kI);
        this.shooter2.config_kD(0, Constants.Shooter.kD);
        this.shooter2.config_kF(0, Constants.Shooter.kF);
    }

    private void setPower(Double pwr) {
        this.shooter1.set(ControlMode.PercentOutput, pwr);
        this.shooter2.set(ControlMode.PercentOutput, pwr);
    }

    private void setVelocity(Double velocity) {
        this.shooter1.set(ControlMode.Velocity, velocity); // -21600 -19800
        this.shooter2.set(ControlMode.Velocity, velocity);
    }

    public void shoot(Double pwr) {
        this.setPower(pwr);
    }

    public void shootVelocity(Double vel) {
        this.setVelocity(vel);
    } 

    public void closeEdgeOfTrench() {
        this.setVelocity(18500.0);
    }

    public void farEdgeOfTrench() {
        this.setVelocity(25000.0);
    }

    public void stopShooter() {
        this.setPower(0.0);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("[Shooter]-Power", this.shooter1.getMotorOutputPercent());
        SmartDashboard.putNumber("[Shooter]-Velocity", this.shooter1.getSelectedSensorVelocity());
        SmartDashboard.putNumber("[Shooter]-Error", this.shooter1.getClosedLoopError());
    }

}