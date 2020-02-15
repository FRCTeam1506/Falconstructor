package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.commands.Intake.IntakeDouble;

public class Intake extends SubsystemBase {

    public WPI_VictorSPX intake = new WPI_VictorSPX(Constants.Intake.INTAKE_ID.getID());

    public Solenoid xfactor = new Solenoid(0);

    private boolean state = false;

    public Intake() {
        this.intake.configFactoryDefault();
    }

    public void intake(double power) {
        this.intake.set(ControlMode.PercentOutput, power);
    }

    public void retract() {
        this.xfactor.set(false);
        state = false;
    }

    public void extend() {
        this.xfactor.set(true);
        state = true;
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("[Intake]-Power", this.intake.get());
        SmartDashboard.putString("[Solenoid]-State", this.state ? "Extended" : "Retracted"); 
    }
}