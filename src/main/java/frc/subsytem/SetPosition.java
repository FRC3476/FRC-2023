package frc.subsytem;

import frc.robot.Constants;

public class SetPosition extends AbstractSubsystem {

    public SetPosition(int period, int loggingInterval) {
        super(Constants.SET_POSITION_PERIOD, 5);
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

    public void setPosition(State state) {
        double elevatorHeight = Elevator.getInstance().getPosition();
        Grabber.getInstance().setPosition(state.grabberPosition);
        if (state == State.MIDDLE_STATE) {
            while (elevatorHeight < state.elevatorPosition) {
                TelescopingArm.getInstance().setPosition(Constants.MIDDLE_NODE_DISTANCE - Constants.GRABBER_LENGTH
                        - Constants.ELEVATOR_DISTANCE_MIDDLE_HEIGHT);
                elevatorHeight = Elevator.getInstance().getPosition();
            }
            TelescopingArm.getInstance().setPosition(state.telescopingArmPosition);
            Grabber.getInstance().setGrabState(Grabber.GrabState.OPEN);
        }

        if (state == State.TOP_STATE) {
            while (elevatorHeight < State.MIDDLE_STATE.elevatorPosition) {
                TelescopingArm.getInstance().setPosition(Constants.MIDDLE_NODE_DISTANCE - Constants.GRABBER_LENGTH
                        - Constants.ELEVATOR_DISTANCE_MIDDLE_HEIGHT);
                elevatorHeight = Elevator.getInstance().getPosition();
            }
            while (elevatorHeight < state.elevatorPosition) {
                TelescopingArm.getInstance().setPosition(Constants.TOP_NODE_DISTANCE - Constants.GRABBER_LENGTH
                        - Constants.ELEVATOR_DISTANCE_TOP_HEIGHT);
                elevatorHeight = Elevator.getInstance().getPosition();
            }
            TelescopingArm.getInstance().setPosition(state.telescopingArmPosition);
            Grabber.getInstance().setGrabState(Grabber.GrabState.OPEN);
        }

        if (state == State.BOTTOM_STATE) {
            if (previousState == State.TOP_STATE) {
                while (elevatorHeight > State.MIDDLE_STATE.elevatorPosition) {
                    TelescopingArm.getInstance().setPosition(State.MIDDLE_STATE.telescopingArmPosition);
                    elevatorHeight = Elevator.getInstance().getPosition();
                }
            }
            while (elevatorHeight > State.BOTTOM_STATE.elevatorPosition) {
                TelescopingArm.getInstance().setPosition(State.BOTTOM_STATE.telescopingArmPosition);
                elevatorHeight = Elevator.getInstance().getPosition();
            }
        }
        previousState = state;
    }
}
