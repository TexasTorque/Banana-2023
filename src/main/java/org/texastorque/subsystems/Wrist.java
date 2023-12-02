package org.texastorque.subsystems;

import org.texastorque.Debug;
import org.texastorque.Ports;
import org.texastorque.Subsystems;
import org.texastorque.torquelib.auto.TorqueSequence;
import org.texastorque.torquelib.auto.commands.TorqueRun;
import org.texastorque.torquelib.auto.commands.TorqueWaitUntil;
import org.texastorque.torquelib.base.TorqueMode;
import org.texastorque.torquelib.base.TorqueState;
import org.texastorque.torquelib.base.TorqueStatorSubsystem;
import org.texastorque.torquelib.motors.TorqueNEO;
import org.texastorque.torquelib.util.TorqueMath;
import edu.wpi.first.math.controller.PIDController;

public class Wrist extends TorqueStatorSubsystem<Wrist.State> implements Subsystems {
    private static volatile Wrist instance;

    public static enum State implements TorqueState {
        UP(31), RIGHT(0), DOWN(-35), AUTO_ORIENT, ROTATE_UP, ROTATE_RIGHT;

        double value;

        private State(final double value) {
            this.value = value;
        }

        private State() {}
    }


    public static final class RotateWrist extends TorqueSequence implements Subsystems {
        public RotateWrist(State desired) {
            addBlock(new TorqueRun(() -> elevator.setState(Elevator.State.SWAP_INTAKE)));
            addBlock(new TorqueWaitUntil(() -> elevator.isAtState()));
            addBlock(new TorqueRun(() -> wrist.setState(desired)));
            addBlock(new TorqueWaitUntil(() -> wrist.isAtState(desired)));
            addBlock(new TorqueRun(() -> elevator.setState(Elevator.State.INTAKE)));
        }
    }

    public boolean isAtState() {
        return TorqueMath.toleranced(wrist.getPosition(), desiredState.value, .3);
    }

    public boolean isAtState(State state) {
        return TorqueMath.toleranced(wrist.getPosition(), state.value, .3);
    }

    private TorqueNEO wrist;
    private PIDController controller;

    private RotateWrist rotateUp, rotateRight;

    public Wrist() {
        super(State.RIGHT);

        wrist = new TorqueNEO(Ports.WRIST);
        wrist.setBreakMode(false);
        wrist.setVoltageCompensation(12.6);
        wrist.setConversionFactors(1, 1);

        controller = new PIDController(1, 0, 0);

        wrist.setCurrentLimit(20);
        wrist.burnFlash();

        rotateUp = new RotateWrist(State.UP);
        rotateRight = new RotateWrist(State.RIGHT);
    }



    @Override
    public void initialize(TorqueMode mode) {}


    @Override
    public void update(TorqueMode mode) {
        Debug.log("Wrist Early State", desiredState.toString());
        Debug.log("Rotate Wrist Up Ended", rotateUp.hasEnded());
        Debug.log("Rotate Wrist Right Ended", rotateRight.hasEnded());

        if (!rotateRight.hasEnded())
            desiredState = State.ROTATE_RIGHT;
        else if (!rotateUp.hasEnded())
            desiredState = State.ROTATE_UP;

        if (desiredState == State.ROTATE_UP)
            rotateUp.run();
        else if (desiredState == State.ROTATE_RIGHT)
            rotateRight.run();
        else {
            rotateUp.reset();
            rotateRight.reset();
        }

        if (elevator.getState() == Elevator.State.INTAKE && desiredState == State.DOWN)
            desiredState = State.UP;


        wrist.setVolts(controller.calculate(wrist.getPosition(), desiredState.value));

        Debug.log("Current Position", wrist.getPosition());
        Debug.log("Wrist State", desiredState.toString());
        Debug.log("Wrist At State", isAtState());
    }

    public static synchronized final Wrist getInstance() {
        return instance == null ? instance = new Wrist() : instance;
    }
}
