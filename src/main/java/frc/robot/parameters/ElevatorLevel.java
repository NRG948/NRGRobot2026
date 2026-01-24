package frc.robot.parameters;

public enum ElevatorLevel {
    // Levels: Stowed, Level 1, Level 2, Level 3
    // TODO: add heights for every level
    STOWED(0),
    L1(0),
    L2(0),
    L3(0);

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
