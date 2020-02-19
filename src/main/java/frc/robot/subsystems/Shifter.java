package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shifter extends SubsystemBase {

    private final Solenoid shifter = new Solenoid(0);

    public Shifter() {}

    public void setToLowGear() {
        if(getState() != "Low") this.shifter.set(true);
    }

    public void setToHighGear() {
        if(getState() != "High") this.shifter.set(false);
    }

    public String getState() {
        return shifter.get() ? "Low" : "High";
    }

    @Override
    public void periodic() {
        SmartDashboard.putString("[Shifter]-Gear", getState());
    }
}