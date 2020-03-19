package frc.robot.subsystems;

import frc.robot.Constants;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VertIndexer extends SubsystemBase {

    private final TalonFX indexer = new TalonFX(Constants.VertIndexer.INDEXER_ID.getID());
    private final SupplyCurrentLimitConfiguration supplyCurrentLimitConfiguration = new SupplyCurrentLimitConfiguration(true, 20.0, 20.0, 0);

    private NetworkTableEntry indexerVoltageWidget;

    public VertIndexer() {
        this.indexer.configFactoryDefault();
        this.indexer.setInverted(false);
        this.indexer.configSupplyCurrentLimit(supplyCurrentLimitConfiguration);

        dashboard();
    }

    public void up() {
        this.indexer.set(ControlMode.PercentOutput, 0.9);
    }

    public void down() {
        this.indexer.set(ControlMode.PercentOutput, -0.9);
    }

    public void stop() {
        this.indexer.set(ControlMode.PercentOutput, 0.0);
    }
    
    private void dashboard() {
        ShuffleboardTab tab = Shuffleboard.getTab("Vertical-Indexer");
        tab.add(this);
        tab.addNumber("Voltage", () -> this.indexer.getStatorCurrent());

        this.indexerVoltageWidget = tab.add("Voltage", 0.0).withWidget(BuiltInWidgets.kGraph).getEntry();
    }

    @Override
    public void periodic() {
        this.indexerVoltageWidget.setNumber(this.indexer.getStatorCurrent());
    }

}