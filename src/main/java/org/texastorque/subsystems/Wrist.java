package org.texastorque.subsystems;

import org.texastorque.Debug;
import org.texastorque.Ports;
import org.texastorque.Subsystems;
import org.texastorque.torquelib.auto.TorqueSequence;
import org.texastorque.torquelib.auto.commands.TorqueRun;
import org.texastorque.torquelib.auto.commands.TorqueRunnableSwitch;
import org.texastorque.torquelib.auto.commands.TorqueWaitTime;
import org.texastorque.torquelib.base.TorqueMode;
import org.texastorque.torquelib.base.TorqueState;
import org.texastorque.torquelib.base.TorqueStatorSubsystem;
import org.texastorque.torquelib.motors.TorqueNEO;
import com.revrobotics.CANSparkMax;

public class Wrist extends TorqueStatorSubsystem<Wrist.State> implements Subsystems {
    private static volatile Wrist instance;

    public static enum State implements TorqueState {
        UP(0), LEFT(5), RIGHT(1), DOWN(10), AUTO;

        double value;

        private State(final double value) {
            this.value = value;
        }

        private State() {}
    }

    public static final class AutoOrientWrist extends TorqueSequence implements Subsystems {
        public AutoOrientWrist() {
            addBlock(new TorqueRun(() -> wrist.setState(State.UP)));
            addBlock(new TorqueWaitTime((1)));
            addBlock(new TorqueRunnableSwitch(wrist::seesTape,
                    new TorqueRun(() -> wrist.setState(State.UP)),
                    new TorqueRun(() -> wrist.setState(State.DOWN))));

        }
    }

    public boolean seesTape() {
        return false;
    }

    private TorqueNEO wrist;

    private AutoOrientWrist autoOrientWrist;

    public Wrist() {
        super(State.UP);

        wrist = new TorqueNEO(Ports.WRIST);
        wrist.setBreakMode(true);
        wrist.setVoltageCompensation(12.6);
        wrist.setConversionFactors(1, 1);
        wrist.configurePIDF(1, 0, 0, 0);
        wrist.setPIDFeedbackDevice(wrist.encoder);
        wrist.burnFlash();

        autoOrientWrist = new AutoOrientWrist();
    }

    @Override
    public void initialize(TorqueMode mode) {}

    @Override
    public void update(TorqueMode mode) {
        if (desiredState == State.AUTO)
            autoOrientWrist.run();
        else {
            wrist.setPIDReference(desiredState.value, CANSparkMax.ControlType.kPosition);
            autoOrientWrist.reset();
        }

        Debug.log("Current Position", wrist.getPosition());
    }



    public static synchronized final Wrist getInstance() {
        return instance == null ? instance = new Wrist() : instance;
    }
}
