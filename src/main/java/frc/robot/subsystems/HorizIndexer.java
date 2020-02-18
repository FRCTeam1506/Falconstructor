package frc.robot.subsystems;

import frc.robot.Constants;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class HorizIndexer extends SubsystemBase {

    private final TalonFX leftIndexer = new TalonFX(Constants.HorizIndexer.LEFT_INDEXER_ID.getID());
    private final TalonFX rightIndexer = new TalonFX(Constants.HorizIndexer.RIGHT_INDEXER_ID.getID());

    public HorizIndexer() {
        this.leftIndexer.configFactoryDefault();
        this.rightIndexer.configFactoryDefault();

        this.leftIndexer.setInverted(true);
        this.rightIndexer.setInverted(false);
    }

    public void run() {
        this.leftIndexer.set(ControlMode.PercentOutput, 0.85);
        this.rightIndexer.set(ControlMode.PercentOutput, 0.6);
    }

    public void rev() {
        this.leftIndexer.set(ControlMode.PercentOutput, -0.6);
        this.rightIndexer.set(ControlMode.PercentOutput, -0.85);
    }

    @Override
    public void periodic() {}

}