package org.texastorque;

import org.texastorque.subsystems.*;
import org.texastorque.torquelib.base.TorqueInput;
import org.texastorque.torquelib.control.TorqueBoolSupplier;
import org.texastorque.torquelib.control.TorqueClickSupplier;
import org.texastorque.torquelib.control.TorqueRequestableTimeout;
import org.texastorque.torquelib.control.TorqueToggleSupplier;
import org.texastorque.torquelib.sensors.TorqueController;
import org.texastorque.torquelib.util.TorqueMath;

public final class Input extends TorqueInput<TorqueController> implements Subsystems {
    private static volatile Input instance;

    private final static double DEADBAND = 0.125;

    private final TorqueBoolSupplier resetGyro, speedDown, speedUp, rotationLock, stow, runIntake,
            runOuttake, mid, high, wristUp, wristDown, wristLeft, wristRight, autoOrientWrist;

    private final TorqueRequestableTimeout driverRumbleTimeout, operatorRumbleTimeout;


    private Input() {
        driver = new TorqueController(0, 0.1);
        operator = new TorqueController(1, 0.1);

        resetGyro = new TorqueClickSupplier(driver::isRightCenterButtonDown);
        speedDown = new TorqueClickSupplier(driver::isLeftBumperDown);
        speedUp = new TorqueClickSupplier(driver::isRightBumperDown);
        rotationLock = new TorqueToggleSupplier(driver::isAButtonDown);

        stow = new TorqueClickSupplier(operator::isAButtonDown);
        runIntake = new TorqueClickSupplier(operator::isRightTriggerDown);
        runOuttake = new TorqueClickSupplier(operator::isLeftTriggerDown);
        mid = new TorqueClickSupplier(operator::isBButtonDown);
        high = new TorqueClickSupplier(operator::isYButtonDown);
        wristUp = new TorqueClickSupplier(operator::isDPADUpDown);
        wristDown = new TorqueClickSupplier(operator::isDPADDownDown);
        wristLeft = new TorqueClickSupplier(operator::isDPADLeftDown);
        wristRight = new TorqueClickSupplier(operator::isDPADRightDown);
        autoOrientWrist = new TorqueClickSupplier(operator::isXButtonDown);

        driverRumbleTimeout = new TorqueRequestableTimeout();
        operatorRumbleTimeout = new TorqueRequestableTimeout();
    }

    @Override
    public final void update() {
        updateWrist();
        updateIntake();
        updateElevator();
        updateDrivebase();
        updateRumble();
    }

    public void updateWrist() {
        wristUp.onTrue(() -> wrist.setState(Wrist.State.UP));
        wristDown.onTrue(() -> wrist.setState(Wrist.State.DOWN));
        wristLeft.onTrue(() -> wrist.setState(Wrist.State.LEFT));
        wristRight.onTrue(() -> wrist.setState(Wrist.State.RIGHT));
        autoOrientWrist.onTrue(() -> wrist.setState(Wrist.State.AUTO));
    }

    public void updateIntake() {
        runIntake.onTrue(() -> intake.setState(Intake.State.INTAKE));
        runOuttake.onTrue(() -> intake.setState(Intake.State.OUTTAKE));
    }

    public void updateElevator() {
        stow.onTrue(() -> elevator.setState(Elevator.State.STOW));
        runIntake.onTrue(() -> elevator.setState(Elevator.State.INTAKE));
        mid.onTrue(() -> elevator.setState(Elevator.State.MID));
        high.onTrue(() -> elevator.setState(Elevator.State.HIGH));
    }

    public void updateDrivebase() {
        resetGyro.onTrue(() -> drivebase.resetGyro());
        speedDown.onTrue(() -> drivebase.speedSetting.shiftDown());
        speedUp.onTrue(() -> drivebase.speedSetting.shiftUp());

        drivebase.rotationLock = rotationLock.get();

        final double xVelocity = TorqueMath.scaledLinearDeadband(driver.getLeftYAxis(), DEADBAND)
                * Drivebase.MAX_VELOCITY_TELEOP;
        final double yVelocity = TorqueMath.scaledLinearDeadband(driver.getLeftXAxis(), DEADBAND)
                * Drivebase.MAX_VELOCITY_TELEOP;
        final double rotationVelocity =
                TorqueMath.scaledLinearDeadband(driver.getRightXAxis(), DEADBAND)
                        * Drivebase.MAX_ANGULAR_VELOCITY;

        drivebase.setInputSpeeds(xVelocity, yVelocity, rotationVelocity);
    }

    public void updateRumble() {
        driver.setRumble(driverRumbleTimeout.get());
        operator.setRumble(operatorRumbleTimeout.get());
    }

    public void setDriverRumbleFor(final double duration) {
        driverRumbleTimeout.set(duration);
    }

    public void setOperatorRumbleFor(final double duration) {
        operatorRumbleTimeout.set(duration);
    }

    public static final synchronized Input getInstance() {
        return instance == null ? instance = new Input() : instance;
    }
}
