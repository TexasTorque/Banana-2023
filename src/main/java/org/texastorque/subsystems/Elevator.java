package org.texastorque.subsystems;

import org.texastorque.Debug;
import org.texastorque.Ports;
import org.texastorque.Subsystems;
import org.texastorque.torquelib.auto.TorqueSequence;
import org.texastorque.torquelib.auto.commands.TorqueContinuous;
import org.texastorque.torquelib.auto.commands.TorqueRun;
import org.texastorque.torquelib.auto.commands.TorqueWaitUntil;
import org.texastorque.torquelib.base.TorqueMode;
import org.texastorque.torquelib.base.TorqueState;
import org.texastorque.torquelib.base.TorqueStatorSubsystem;
import org.texastorque.torquelib.motors.TorqueNEO;
import org.texastorque.torquelib.util.TorqueMath;

public class Elevator extends TorqueStatorSubsystem<Elevator.State> implements Subsystems {
    private static volatile Elevator instance;

    private static final double MIN_HEIGHT = 0.5, MAX_HEIGHT = 800;

    public static enum State implements TorqueState {
        INTAKE(.5), SWAP_INTAKE(40), MID(450), MID_DUNK(350), HIGH(795), HIGH_DUNK(700);

        double height;

        private State(final double height) {
            this.height = TorqueMath.constrain(height, MIN_HEIGHT, MAX_HEIGHT);
        }
    }

    public static final class DunkNScore extends TorqueSequence implements Subsystems {
        public DunkNScore(State desired) {
            addBlock(new TorqueRun(() -> elevator.setState(desired)));
            addBlock(new TorqueWaitUntil(() -> elevator.isAtState()));
            addBlock(new TorqueContinuous(() -> intake.setState(Intake.State.OUTTAKE)));
        }
    }

    private final TorqueNEO elevator;

    private double operatorAdjustment = 0, desiredHeight = 0;

    private final DunkNScore midDunkNScore, highDunkNScore;

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

        midDunkNScore = new DunkNScore(State.MID_DUNK);
        highDunkNScore = new DunkNScore(State.HIGH_DUNK);
    }

    @Override
    public void initialize(TorqueMode mode) {}

    public void setOperatorAdjustment(final double adj) {
        operatorAdjustment = adj;
    }

    @Override
    public void update(TorqueMode mode) {
        if (desiredState == State.MID_DUNK)
            midDunkNScore.run();
        else if (desiredState == State.HIGH_DUNK)
            highDunkNScore.run();

        desiredHeight = TorqueMath.constrain(desiredState.height + operatorAdjustment * 50,
                MIN_HEIGHT, MAX_HEIGHT);

        if (!TorqueMath.toleranced(elevator.getPosition(), desiredHeight, .2))
            elevator.setPosition(desiredHeight);
        else
            elevator.setVolts(0);

        Debug.log("Current Height", elevator.getPosition());
        Debug.log("Desired Height", desiredState.height);
        Debug.log("Elevator State", desiredState.toString());
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

    public boolean isAtIntake() {
        return desiredState == State.INTAKE || desiredState ==State.SWAP_INTAKE;
    }

    public static synchronized final Elevator getInstance() {
        return instance == null ? instance = new Elevator() : instance;
    }
}
