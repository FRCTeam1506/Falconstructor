package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Limelight extends SubsystemBase {

    private Double x, y, area;
    private boolean targetFound;
    private Integer pipeline;

    private NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");

    public Limelight() {}

    @Override
    public void periodic() {
        this.x = table.getEntry("tx").getDouble(0.0);
        this.y = table.getEntry("ty").getDouble(0.0);
        this.area = table.getEntry("ta").getDouble(0.0);
        this.targetFound = table.getEntry("tv").getNumber(0).intValue() == 1 ? true : false;
        this.pipeline = table.getEntry("pipeline").getNumber(0).intValue();
    }

    public void setPipeline(int index) {
        this.table.getEntry("pipeline").setNumber(index);
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

    public boolean isTargetFound() {
        return targetFound;
    }
}