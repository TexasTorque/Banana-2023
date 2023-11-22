package org.texastorque.subsystems;

import org.texastorque.Debug;
import org.texastorque.Input;
import org.texastorque.Ports;
import org.texastorque.Subsystems;
import org.texastorque.torquelib.base.TorqueMode;
import org.texastorque.torquelib.base.TorqueState;
import org.texastorque.torquelib.base.TorqueStatorSubsystem;
import org.texastorque.torquelib.control.TorqueRequestableTimeout;
import org.texastorque.torquelib.motors.TorqueNEO;

public class Intake extends TorqueStatorSubsystem<Intake.State> implements Subsystems {
    private static volatile Intake instance;

    public static enum State implements TorqueState {
        OFF(0), INTAKE(-5), OUTTAKE(12);

        public final double rollerSpeed;

        private State(final double rollerSpeed) {
            this.rollerSpeed = rollerSpeed;
        }
    }

    private final double CURRENT_SPIKE = 10;

    private final TorqueNEO rollers;

    private final TorqueRequestableTimeout spikeTimeout;

    private boolean spiked = false;


    public Intake() {
        super(State.OFF);

        rollers = new TorqueNEO(Ports.INTAKE_ROLLERS);
        rollers.setVoltageCompensation(12.6);
        rollers.setCurrentLimit(10);
        rollers.setBreakMode(false);
        spikeTimeout = new TorqueRequestableTimeout();
    }

    @Override
    public void initialize(final TorqueMode mode) {}


    @Override
    public void update(final TorqueMode mode) {
        Debug.log("Rollers Current", rollers.getCurrent());
        Debug.log("Intake State", desiredState.toString());


        if (desiredState == State.INTAKE) {
            if (!spikeTimeout.get() && rollers.getCurrent() >= (CURRENT_SPIKE)) {
                Input.getInstance().setDriverRumbleFor(.2);
                Input.getInstance().setOperatorRumbleFor(.2);
                desiredState = State.OFF;
                spiked = true;
            }
        } else {
            spikeTimeout.set(1);
            spiked = false;
        }


        if (spiked)
            desiredState = State.OFF;

        rollers.setVolts(-desiredState.rollerSpeed);

        if (mode.isTeleop())
            desiredState = State.OFF;
    }

    public static synchronized final Intake getInstance() {
        return instance == null ? instance = new Intake() : instance;
    }
};
