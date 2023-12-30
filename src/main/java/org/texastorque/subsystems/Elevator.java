package org.texastorque.subsystems;

import org.texastorque.Debug;
import org.texastorque.Ports;
import org.texastorque.Subsystems;
import org.texastorque.torquelib.base.TorqueMode;
import org.texastorque.torquelib.base.TorqueState;
import org.texastorque.torquelib.base.TorqueStatorSubsystem;
import org.texastorque.torquelib.motors.TorqueNEO;
import org.texastorque.torquelib.util.TorqueMath;

public class Elevator extends TorqueStatorSubsystem<Elevator.State> implements Subsystems {
    private static volatile Elevator instance;

    public static enum State implements TorqueState {
        INTAKE(0.5), SWAP_INTAKE(40), THROW(300), MID(450), MID_DUNK(350), HIGH(795), HIGH_DUNK(700);

        private final double MIN_HEIGHT = 0.5, MAX_HEIGHT = 800;

        double height;

        private State(final double height) {
            this.height = TorqueMath.constrain(height, MIN_HEIGHT, MAX_HEIGHT);
        }
    }

    private final TorqueNEO elevator;

    public Elevator() {
        super(State.INTAKE);

        elevator = new TorqueNEO(Ports.ELEVATOR);
        elevator.setBreakMode(true);
        elevator.setVoltageCompensation(12.6);
        elevator.setConversionFactors(1, 1);
        elevator.setPIDFeedbackDevice(elevator.encoder);
        elevator.configurePIDF(10, 0, 0, 0);
        elevator.setCurrentLimit(30);
        elevator.burnFlash();
    }

    @Override
    public void initialize(TorqueMode mode) {}

    @Override
    public void update(TorqueMode mode) {
        if (!TorqueMath.toleranced(elevator.getPosition(), desiredState.height, .2))
            elevator.setPosition(desiredState.height);
        else
            elevator.setVolts(0);

        Debug.log("Current Height", elevator.getPosition());
        Debug.log("Desired Height", desiredState.height);
        Debug.log("Elevator State", desiredState.toString());

        Debug.log("isAtState", isAtState());
    }

    public boolean isLowCG() {
        return elevator.getPosition() <= 500;
    }

    public boolean isAtState() {
        return TorqueMath.toleranced(elevator.getPosition(), desiredState.height, .2);
    }

    public boolean isAtState(State state) {
        return TorqueMath.toleranced(elevator.getPosition(), state.height, .2);
    }

    public boolean isAtMid() {
        return isAtState(State.MID) || isAtState(State.MID_DUNK);
    }

    public static synchronized final Elevator getInstance() {
        return instance == null ? instance = new Elevator() : instance;
    }
}
