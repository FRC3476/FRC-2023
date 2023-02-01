package frc.subsytem;

import frc.robot.Constants;

public class SetPosition extends AbstractSubsystem {

    Elevator elevator;
    TelescopingArm telescopingArm;
    Grabber grabber;

    public SetPosition(int period, int loggingInterval) {
        super(Constants.SET_POSITION_PERIOD, 5);
        elevator = Elevator.getInstance();
        telescopingArm = TelescopingArm.getInstance();
        grabber = Grabber.getInstance();
    }

    public enum State {
        BOTTOM_STATE(0, 0, 0),
        MIDDLE_STATE(0, 0, 0),
        TOP_STATE(0, 0, 0);
        final double elevatorPosition, telescopingArmPosition, grabberPosition;

        State(double elevatorPosition, double telescopingArmPosition, double grabberPosition) {
            this.elevatorPosition = elevatorPosition;
            this.telescopingArmPosition = telescopingArmPosition;
            this.grabberPosition = grabberPosition;
        }
    }

    State previousState;

    /**
     * @param state - State enum containing an elevator position, telescoping arm position, and a grabber position
     */
    public void setPosition(State state) {
        double elevatorHeight = elevator.getPosition();

        //The grabber usually won't be restricted by anything, so we move it first
        grabber.setPosition(state.grabberPosition);

        if (state == State.BOTTOM_STATE) {
            //Checks if the robot is at the top node or the middle node
            if (previousState == State.TOP_STATE) {
                //This moves the telescoping arm while the elevator is above the middle node
                while (elevatorHeight > State.MIDDLE_STATE.elevatorPosition) {
                    telescopingArm.setPosition(State.MIDDLE_STATE.telescopingArmPosition);
                    elevatorHeight = elevator.getPosition();
                }
            }
            //This moves the telescoping arm while the elevator isn't in the right spot
            while (elevatorHeight > State.BOTTOM_STATE.elevatorPosition) {
                telescopingArm.setPosition(State.BOTTOM_STATE.telescopingArmPosition);
                elevatorHeight = elevator.getPosition();
            }
        } else {
            while (elevatorHeight < State.MIDDLE_STATE.elevatorPosition) {
                telescopingArm.setPosition(Constants.MIDDLE_NODE_DISTANCE - Constants.GRABBER_LENGTH
                        - Constants.ELEVATOR_DISTANCE_MIDDLE_HEIGHT);
                elevatorHeight = elevator.getPosition();
            }
            if (state == State.MIDDLE_STATE) {
                telescopingArm.setPosition(state.telescopingArmPosition);
                grabber.setGrabState(Grabber.GrabState.OPEN);
            }
            if (state == State.TOP_STATE) {
                //This moves the telescoping arm while the elevator isn't in the right spot
                while (elevatorHeight < state.elevatorPosition) {
                    telescopingArm.setPosition(Constants.TOP_NODE_DISTANCE - Constants.GRABBER_LENGTH
                            - Constants.ELEVATOR_DISTANCE_TOP_HEIGHT);
                    elevatorHeight = elevator.getPosition();
                }
                telescopingArm.setPosition(state.telescopingArmPosition);
                grabber.setGrabState(Grabber.GrabState.OPEN);
            }
            previousState = state;
        }
    }
}
