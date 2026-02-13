package frc.robot.parameters;

public enum ElevatorLevel {
    // Levels: Stowed, Level 1, Level 2, Level 3
    // TODO: add heights for every bar level
    STOWED(0),
    Bar1(5),
    Bar2(10),
    Bar3(20);

    // variables
    private final double elevatorHeight;
    

    // constructor 
    private ElevatorLevel (double height) {
        this.elevatorHeight = height;
    }

    // getters
    public double getElevatorHeight() {
        return elevatorHeight;
    }

}
