package org.texastorque.subsystems;

import org.texastorque.Debug;
import org.texastorque.Ports;
import org.texastorque.Subsystems;
import org.texastorque.torquelib.auto.TorqueSequence;
import org.texastorque.torquelib.auto.commands.TorqueRun;
import org.texastorque.torquelib.auto.commands.TorqueRunWhile;
import org.texastorque.torquelib.auto.commands.TorqueWaitUntil;
import org.texastorque.torquelib.base.TorqueMode;
import org.texastorque.torquelib.base.TorqueState;
import org.texastorque.torquelib.base.TorqueStatorSubsystem;
import org.texastorque.torquelib.motors.TorqueNEO;
import org.texastorque.torquelib.util.TorqueMath;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;
import edu.wpi.first.math.controller.PIDController;

public class Wrist extends TorqueStatorSubsystem<Wrist.State> implements Subsystems {
    private static volatile Wrist instance;

    public static enum State implements TorqueState {
        UP(0.24), RIGHT(0), DOWN(-0.25), ROTATE_UP, ROTATE_RIGHT;

        double value;

        private State(final double value) {
            this.value = value;
        }

        private State() {}
    }

    public static final class AutoRotateWrist extends TorqueSequence implements Subsystems {
        public AutoRotateWrist(State desired) {
            addBlock(new TorqueRun(() -> elevator.setState(Elevator.State.SWAP_INTAKE)));
            addBlock(new TorqueWaitUntil(() -> elevator.isAtState()));
            addBlock(new TorqueRunWhile(new TorqueRun(() -> wrist.setState(desired)),
                    () -> !wrist.isAtState(desired)));
            addBlock(new TorqueRun(() -> elevator.setState(Elevator.State.INTAKE)));
        }
    }

    public boolean isAtState() {
        return TorqueMath.toleranced(encoder.getPosition() - WRIST_OFFSET, desiredState.value, .1);
    }

    public boolean isAtState(State state) {
        return TorqueMath.toleranced(encoder.getPosition() - WRIST_OFFSET, state.value, .1);
    }

    private final PIDController controller;

    private final TorqueNEO wrist;

    private final AutoRotateWrist autoRotateUp, autoRotateRight;

    private final AbsoluteEncoder encoder;

    private final double WRIST_OFFSET = 0.478;

    public Wrist() {
        super(State.RIGHT);

        wrist = new TorqueNEO(Ports.WRIST);
        wrist.setBreakMode(true);
        wrist.setVoltageCompensation(12.6);
        wrist.setConversionFactors(1, 1);
        encoder = wrist.getAbsoluteEncoder(Type.kDutyCycle);

        controller = new PIDController(30, 0, 0);

        wrist.setCurrentLimit(20);
        wrist.burnFlash();

        autoRotateUp = new AutoRotateWrist(State.UP);
        autoRotateRight = new AutoRotateWrist(State.RIGHT);
    }

    @Override
    public void initialize(TorqueMode mode) {}

    @Override
    public void update(TorqueMode mode) {
        if (desiredState == State.ROTATE_UP)
            autoRotateUp.run();
        else if (desiredState != State.UP)
            autoRotateUp.reset();

        if (desiredState == State.ROTATE_RIGHT)
            autoRotateRight.run();
        else
            autoRotateRight.reset();

        boolean runningSeq = desiredState == State.ROTATE_RIGHT || desiredState == State.ROTATE_UP;

        wrist.setVolts(runningSeq ? 0
                : controller.calculate(encoder.getPosition() - WRIST_OFFSET, desiredState.value));

        Debug.log("Wrist State", desiredState.toString());
        Debug.log("Wrist At State", isAtState());
        Debug.log("Wrist Position", encoder.getPosition() - WRIST_OFFSET);
    }

    public static synchronized final Wrist getInstance() {
        return instance == null ? instance = new Wrist() : instance;
    }
}
