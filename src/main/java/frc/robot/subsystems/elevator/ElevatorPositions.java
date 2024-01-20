package frc.robot.subsystems.elevator;

import frc.robot.Constants.ElevatorConstants;

public enum ElevatorPositions {
    CLIMB_INIT(ElevatorConstants.elevatorClimbInit),
    CLIMB_END(ElevatorConstants.elevatorClimbHome);

    private final int position;

    ElevatorPositions(int position) {
        this.position = position;
    }

    public int getPosition() {
        return this.position;
    }
}