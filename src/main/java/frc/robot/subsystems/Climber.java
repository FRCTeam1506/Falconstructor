package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {

    private DoubleSolenoid arm = new DoubleSolenoid(2,3);
    private TalonFX leftClimber = new TalonFX(3);
    private TalonFX rightClimber = new TalonFX(12);

    public Climber() {
        this.leftClimber.setInverted(true);
        this.rightClimber.setInverted(false);

        this.leftClimber.setNeutralMode(NeutralMode.Brake);
        this.rightClimber.setNeutralMode(NeutralMode.Brake);

        this.leftClimber.config_kP(0, 0.2);
        this.leftClimber.config_kI(0, 0.0);
        this.leftClimber.config_kD(0, 0.0);
        this.leftClimber.config_kF(0, 0.2);

        this.rightClimber.config_kP(0, 0.2);
        this.rightClimber.config_kI(0, 0.0);
        this.rightClimber.config_kD(0, 0.0);
        this.rightClimber.config_kF(0, 0.2);

        this.leftClimber.configMotionCruiseVelocity(100000);
        this.leftClimber.configMotionAcceleration(30000);

        this.rightClimber.configMotionCruiseVelocity(100000);
        this.rightClimber.configMotionAcceleration(30000);

        DEFAULT();
        reset();
        dashboard();
    }

    public void DEFAULT() {
        retract();
        this.leftClimber.set(ControlMode.PercentOutput, 0.0);
        this.rightClimber.set(ControlMode.PercentOutput, 0.0);
    }

    public void extend() {
        if(getState() != Value.kForward) this.arm.set(Value.kForward);
    }

    public void retract() {
        if(getState() != Value.kReverse) this.arm.set(Value.kReverse);
    }

    public void setInitial() {
        this.leftClimber.set(ControlMode.MotionMagic, 0.0);
        this.rightClimber.set(ControlMode.MotionMagic, 0.0);
    }

    public void setClimbing() {
        this.leftClimber.set(ControlMode.MotionMagic, -300000.0);
        this.rightClimber.set(ControlMode.MotionMagic, -300000.0);
    }

    public void controlClimber(double leftPower, double rightPower) {

        if(leftPower > 0.1 || leftPower < -0.1) this.leftClimber.set(ControlMode.PercentOutput, leftPower); else this.leftClimber.set(ControlMode.PercentOutput, 0.0);
        if(rightPower > 0.1 || rightPower < -0.1) this.rightClimber.set(ControlMode.PercentOutput, rightPower); else this.rightClimber.set(ControlMode.PercentOutput, 0.0);
        
        // if(this.leftClimber.getSelectedSensorPosition() <= 0) {
        //     this.leftClimber.set(ControlMode.PercentOutput, leftPower);
        // } else {
        //     this.leftClimber.set(ControlMode.PercentOutput, 0.0);
        // }
        // if(this.rightClimber.getSelectedSensorPosition() <= 0) {
        //     this.rightClimber.set(ControlMode.PercentOutput, rightPower); 
        // } else { 
        //     this.rightClimber.set(ControlMode.PercentOutput, 0.0);
        // }

        /*
        if(this.leftClimber.getSelectedSensorPosition() <= -300000.0) {
            this.leftClimber.set(ControlMode.PercentOutput, -0.1);
        } else if(this.leftClimber.getSelectedSensorPosition() >= 0.0) {
            this.leftClimber.set(ControlMode.PercentOutput, 0.1);
        }

        if(this.rightClimber.getSelectedSensorPosition() <= -300000.0) {
            this.rightClimber.set(ControlMode.PercentOutput, -0.1);
        } else if(this.rightClimber.getSelectedSensorPosition() >= 0.0) {
            this.rightClimber.set(ControlMode.PercentOutput, 0.1);
        }
        */
    }

    public Value getState() {
        return this.arm.get();
    }

    // public TalonFX getLeftClimber() {
    //     return this.leftClimber;
    // }

    // public TalonFX getRightClimber() {
    //     return this.rightClimber;
    // }

    public void reset() {
        this.leftClimber.setSelectedSensorPosition(0);
        this.rightClimber.setSelectedSensorPosition(0);
    }

    private void dashboard() {
        ShuffleboardTab tab = Shuffleboard.getTab("Climber");
        tab.addNumber("Left Encoder", () -> this.leftClimber.getSelectedSensorPosition());
        tab.addNumber("Right Encoder", () -> this.rightClimber.getSelectedSensorPosition());
        tab.addString("Arm State", () -> this.getState().toString());
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Left Encoder", this.leftClimber.getSelectedSensorPosition());
        SmartDashboard.putNumber("Right Encoder", this.rightClimber.getSelectedSensorPosition());
    }

}