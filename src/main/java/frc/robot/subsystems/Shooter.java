package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {

    // Instantiate Hardware
    public TalonFX shooter1 = new TalonFX(Constants.Shooter.SHOOTER_1_ID.getID());
    public TalonFX shooter2 = new TalonFX(Constants.Shooter.SHOOTER_2_ID.getID());

    public Shooter() {
        this.shooter1.configFactoryDefault();
        this.shooter2.configFactoryDefault();

        this.shooter1.setInverted(true);
        this.shooter2.setInverted(false);
    }

    // Subsystem Functions
    public void shoot(Double pwr) {
        shooter1.set(ControlMode.PercentOutput, pwr);
        shooter2.set(ControlMode.PercentOutput, pwr);
    }
    public void fullSend() {
        shooter1.set(ControlMode.PercentOutput, 1);
        shooter2.set(ControlMode.PercentOutput, 1);
    }
}