package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Indexer extends SubsystemBase {

    private WPI_VictorSPX indexer = new WPI_VictorSPX(Constants.Indexer.INDEXER_ID.getID());

    public Indexer() {
        this.indexer.configFactoryDefault();
    }

    public void index(double power) {
        this.indexer.set(ControlMode.PercentOutput, power);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("[Indexer]-Power", this.indexer.get());
    }

}