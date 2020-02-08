package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {

    public WPI_VictorSPX intake = new WPI_VictorSPX(Constants.Intake.INTAKE_ID.getID());

    public Intake() {
        this.intake.configFactoryDefault();
    }

    public void intake(double power) {
        this.intake.set(ControlMode.PercentOutput, power);
    }

    public void intake(double time, double power) {}

}