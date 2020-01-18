package frc.robot.utils;

public class NamedID {

    /*
    *   @param name --- description
    *   @param id   --- port number
    */

    private String name;
    private Integer id;

    public NamedID(String name, Integer id) {
        this.name = name;
        this.id = id;
    }

    public Integer getID() {
        return id;
    }

    public String getName() {
        return name;
    }
}