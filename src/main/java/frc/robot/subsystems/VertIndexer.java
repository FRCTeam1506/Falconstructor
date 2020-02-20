package frc.robot.subsystems;

import frc.robot.Constants;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VertIndexer extends SubsystemBase {

    private final TalonFX indexer = new TalonFX(Constants.VertIndexer.INDEXER_ID.getID());

    public VertIndexer() {
        this.indexer.configFactoryDefault();
        this.indexer.setInverted(false);
    }

    public void up() {
        this.indexer.set(ControlMode.PercentOutput, 0.3);
    }

    public void down() {
        this.indexer.set(ControlMode.PercentOutput, -0.3);
    }

    public void stop() {
        this.indexer.set(ControlMode.PercentOutput, 0.0);
    }

    @Override
    public void periodic() {}

}