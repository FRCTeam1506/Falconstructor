package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Turret extends SubsystemBase {

    // Instantiate Hardware
    public TalonFX turret = new TalonFX(Constants.Turret.TURRET_ID.getID());

    public Turret() {
        this.turret.configFactoryDefault();
        this.turret.setInverted(false);
        // TODO: Motion Magic Configuration
    }

    // Subsystem Functions
    public void goToPos(double pos) {
        this.turret.set(TalonFXControlMode.MotionMagic, pos);
    }

    public void resetEncoders() {
        this.turret.setSelectedSensorPosition(0, 0, 0);
    }
}