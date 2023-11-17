package org.texastorque.subsystems;

import org.texastorque.Debug;
import org.texastorque.Ports;
import org.texastorque.Subsystems;
import org.texastorque.torquelib.base.TorqueMode;
import org.texastorque.torquelib.base.TorqueState;
import org.texastorque.torquelib.base.TorqueStatorSubsystem;
import org.texastorque.torquelib.motors.TorqueNEO;
import com.revrobotics.CANSparkMax;

public class Elevator extends TorqueStatorSubsystem<Elevator.State> implements Subsystems {
    private static volatile Elevator instance;

    public static enum State implements TorqueState {
        STOW(0), INTAKE(1), MID(5), HIGH(10);

        double height;

        private State(final double height) {
            this.height = height;
        }
    }

    private TorqueNEO elevator;

    public Elevator() {
        super(State.STOW);

        elevator = new TorqueNEO(Ports.ELEVATOR);
        elevator.setBreakMode(true);
        elevator.setVoltageCompensation(12.6);
        elevator.setConversionFactors(1, 1);
        elevator.configurePIDF(1, 0, 0, 0);
        elevator.setPIDFeedbackDevice(elevator.encoder);
        elevator.burnFlash();
    }

    @Override
    public void initialize(TorqueMode mode) {}

    @Override
    public void update(TorqueMode mode) {
        elevator.setPIDReference(desiredState.height, CANSparkMax.ControlType.kPosition);
        Debug.log("Current Height", elevator.getPosition());
    }

    public boolean isAtStow() {
        return isAtState(State.STOW);
    }

    public static synchronized final Elevator getInstance() {
        return instance == null ? instance = new Elevator() : instance;
    }
}
