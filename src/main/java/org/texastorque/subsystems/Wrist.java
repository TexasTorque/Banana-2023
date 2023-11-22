package org.texastorque.subsystems;

import java.util.function.BooleanSupplier;
import org.texastorque.Debug;
import org.texastorque.Ports;
import org.texastorque.Subsystems;
import org.texastorque.torquelib.auto.TorqueSequence;
import org.texastorque.torquelib.auto.commands.TorqueRun;
import org.texastorque.torquelib.auto.commands.TorqueRunWhile;
import org.texastorque.torquelib.auto.commands.TorqueRunnableSwitch;
import org.texastorque.torquelib.auto.commands.TorqueWaitTime;
import org.texastorque.torquelib.auto.commands.TorqueWaitUntil;
import org.texastorque.torquelib.base.TorqueMode;
import org.texastorque.torquelib.base.TorqueState;
import org.texastorque.torquelib.base.TorqueStatorSubsystem;
import org.texastorque.torquelib.control.TorqueToggle;
import org.texastorque.torquelib.motors.TorqueNEO;
import org.texastorque.torquelib.util.TorqueMath;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.controller.PIDController;

public class Wrist extends TorqueStatorSubsystem<Wrist.State> implements Subsystems {
    private static volatile Wrist instance;

    public static enum State implements TorqueState {
        // Remove left or right
        UP(31), RIGHT(0), DOWN(-35), AUTO_ORIENT, ROTATE_UP, ROTATE_RIGHT;

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
            // addBlock(new TorqueRunnableSwitch(wrist::seesTape,
            // new TorqueRun(() -> wrist.setState(State.UP)),
            // new TorqueRun(() -> wrist.setState(State.DOWN))));

        }
    }

    public static final class AutoRotateWrist extends TorqueSequence implements Subsystems {
        public AutoRotateWrist(State desired) {
            addBlock(new TorqueRun(() -> elevator.setState(Elevator.State.SWAP_INTAKE)));
            addBlock(new TorqueWaitUntil(() -> elevator.isAtState()));
            addBlock(new TorqueRun(() -> wrist.setState(desired)));
            addBlock(new TorqueWaitUntil(() -> wrist.isAtState()));
            addBlock(new TorqueRun(() -> elevator.setState(Elevator.State.INTAKE)));
        }
    }

    public boolean seesTape() {
        // Add TOAST HSV code here
        return false;
    }

    public boolean isAtState() {
        return TorqueMath.toleranced(wrist.getPosition(), desiredState.value, .3);
    }

    public boolean isNotAtState() {
        return !isAtState();
    }

    public boolean isAtState(State state) {
        return TorqueMath.toleranced(wrist.getPosition(), state.value, .3);
    }

    private TorqueNEO wrist;
    private PIDController controller;

    private AutoOrientWrist autoOrientWrist;

    private AutoRotateWrist autoRotateUp, autoRotateRight;

    public Wrist() {
        super(State.RIGHT);

        wrist = new TorqueNEO(Ports.WRIST);
        wrist.setBreakMode(false);
        wrist.setVoltageCompensation(12.6);
        wrist.setConversionFactors(1, 1);

        controller = new PIDController(1, 0, 0);

        wrist.setCurrentLimit(20);
        wrist.burnFlash();

        autoOrientWrist = new AutoOrientWrist();
        autoRotateUp = new AutoRotateWrist(State.UP);
        autoRotateRight = new AutoRotateWrist(State.RIGHT);
    }



    @Override
    public void initialize(TorqueMode mode) {}

    @Override
    public void update(TorqueMode mode) {
        Debug.log("Wrist Early State", desiredState.toString());

        if (desiredState == State.AUTO_ORIENT)
            autoOrientWrist.run();
        else if (desiredState == State.ROTATE_UP)
            autoRotateUp.run();
        else if (desiredState == State.ROTATE_RIGHT)
            autoRotateRight.run();
        else {
            autoOrientWrist.reset();
            autoRotateUp.reset();
            autoRotateRight.reset();
        }

        if (elevator.isAtState(Elevator.State.INTAKE) && desiredState == State.DOWN) 
            desiredState = State.UP;


        wrist.setVolts(controller.calculate(wrist.getPosition(), desiredState.value));


        Debug.log("Current Position", wrist.getPosition());
        Debug.log("Wrist PID Volts", controller.calculate(wrist.getPosition(), desiredState.value));
        Debug.log("Wrist State", desiredState.toString());
        Debug.log("Wrist At State", isAtState());
    }



    public static synchronized final Wrist getInstance() {
        return instance == null ? instance = new Wrist() : instance;
    }
}
