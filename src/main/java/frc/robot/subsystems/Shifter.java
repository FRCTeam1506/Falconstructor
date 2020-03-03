package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shifter extends SubsystemBase {

    private final Solenoid shifter = new Solenoid(0);

    public Shifter() {
        dashboard();
    }

    public void setToLowGear() {
        if(getState() != "Low") this.shifter.set(true);
    }

    public void setToHighGear() {
        if(getState() != "High") this.shifter.set(false);
    }

    public String getState() {
        return shifter.get() ? "Low" : "High";
    }

    private void dashboard() {
        ShuffleboardTab tab = Shuffleboard.getTab("Shifter");
        tab.addString("State", () -> getState());
    }

    @Override
    public void periodic() {
        SmartDashboard.putString("[Shifter]-Gear", getState());
    }
}