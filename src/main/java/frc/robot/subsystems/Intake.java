package frc.robot.subsystems;

import frc.robot.Constants;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {

    private final TalonFX intake = new TalonFX(Constants.Intake.INTAKE_ID.getID());
    private final Solenoid xfactor = new Solenoid(Constants.Intake.XFACTOR_ID.getID());

    public Intake() {
        this.intake.configFactoryDefault();
        this.intake.setInverted(false);
    }

    public void intakeFwd() {
        this.intake.set(ControlMode.PercentOutput, 0.85);
    }

    public void intakeFwd(double pwr) {
        this.intake.set(ControlMode.PercentOutput, pwr);
    }

    public void intakeRev() {
        this.intake.set(ControlMode.PercentOutput, -0.85);
    }

    public void intakeRev(double pwr) {
        this.intake.set(ControlMode.PercentOutput, -pwr);
    }

    public void retract() {
        this.xfactor.set(false);
    }

    public void extend() {
        this.xfactor.set(true);
    }

    @Override
    public void periodic() {
        SmartDashboard.putString("[Intake]-Xfactor-State", this.xfactor.get() ? "Extended" : "Retracted" );
    }
}