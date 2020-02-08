package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Limelight extends SubsystemBase {

    public boolean aligned, isRefreshed;
    private Double x, y, area, previousX, previousY;
    private boolean targetFound;
    private Integer pipeline;

    private NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");

    public enum Piplelines {
        NearTargeting,
        FarTargeting
    }

    public Limelight() {}

    @Override
    public void periodic() {
        this.x = table.getEntry("tx").getDouble(0.0);
        this.y = table.getEntry("ty").getDouble(0.0);
        this.area = table.getEntry("ta").getDouble(0.0);
        this.targetFound = table.getEntry("tv").getNumber(0).intValue() == 1 ? true : false;
        this.pipeline = table.getEntry("pipeline").getNumber(0).intValue();
        this.aligned = Math.abs(x) < Constants.Limelight.THRESHOLD ? true : false;
        this.isRefreshed = this.x != previousX || this.y != previousY ? true : false;
        this.previousX = this.x;
        this.previousY = this.y;
        // if(Math.abs(x) < Constants.Limelight.THRESHOLD) this.aligned = true;
        // else this.aligned = false;
    }

    public void setPipeline(int index) {
        this.table.getEntry("pipeline").setNumber(index);
    }

    public void setPipeline(Piplelines pipe) {
        switch (pipe) {
            case NearTargeting:
                this.setPipeline(5);
                break;

            case FarTargeting:
                this.setPipeline(6);
                break;
        
            default:
                this.setPipeline(5);
                break;
        }
    }

    public Double getX() {
        return this.x;
    }

    public Double getY() {
        return this.y;
    }

    public Double getArea() {
        return this.area;
    }

    public int getPipeline() {
        return this.pipeline;
    }

    public boolean isRefreshed() {
        return this.isRefreshed;
    }

    public Double getDistance() {
        // inches
        Double h2 = 96.0;
        Double h1 = 30.0;
        Double a1 = 0.0;
        Double a2 = this.y;
        return (h2 - h1) / Math.tan(a1 + a2);
    }

    public boolean isTargetFound() {
        return this.targetFound;
    }

    public boolean isAligned() {
        return this.aligned;
    }
}