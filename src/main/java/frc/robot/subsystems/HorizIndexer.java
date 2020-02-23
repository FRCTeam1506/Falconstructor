package frc.robot.subsystems;

import frc.robot.Constants;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class HorizIndexer extends SubsystemBase {

    private final TalonFX leftIndexer = new TalonFX(Constants.HorizIndexer.LEFT_INDEXER_ID.getID());
    private final TalonFX rightIndexer = new TalonFX(Constants.HorizIndexer.RIGHT_INDEXER_ID.getID());

    private SupplyCurrentLimitConfiguration supplyCurrentLimitConfiguration = new SupplyCurrentLimitConfiguration(true, 20.0, 20.0, 0);

    public HorizIndexer() {
        this.leftIndexer.configFactoryDefault();
        this.rightIndexer.configFactoryDefault();

        this.leftIndexer.setInverted(true);
        this.rightIndexer.setInverted(false);
        this.rightIndexer.setNeutralMode(NeutralMode.Coast);

        this.leftIndexer.configSupplyCurrentLimit(supplyCurrentLimitConfiguration);
        this.rightIndexer.configSupplyCurrentLimit(supplyCurrentLimitConfiguration);
    }

    public void rev() {
        this.leftIndexer.set(ControlMode.PercentOutput, 0.5);
        this.rightIndexer.set(ControlMode.PercentOutput, 0.4);
    }

    public void run() {
        this.leftIndexer.set(ControlMode.PercentOutput, -0.4);
        this.rightIndexer.set(ControlMode.PercentOutput, -0.35);
    }

    public void cycle() {
        this.leftIndexer.set(ControlMode.PercentOutput, -0.4); //-0.4
        this.rightIndexer.set(ControlMode.PercentOutput, -0.15);
    }

    public void rev_cycle() {
        this.leftIndexer.set(ControlMode.PercentOutput, 0.15);
        this.rightIndexer.set(ControlMode.PercentOutput, -0.4);
    }

    public void left() {
        this.leftIndexer.set(ControlMode.PercentOutput, -0.4);
    }

    public void right() {
        this.rightIndexer.set(ControlMode.PercentOutput, -0.35);
    }

    public void stop() {
        this.leftIndexer.set(ControlMode.PercentOutput, 0.0);
        this.rightIndexer.set(ControlMode.PercentOutput, 0.0);
    }

    @Override
    public void periodic() {}

}