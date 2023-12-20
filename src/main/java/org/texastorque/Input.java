package org.texastorque;

import org.texastorque.subsystems.*;
import org.texastorque.torquelib.base.TorqueInput;
import org.texastorque.torquelib.control.TorqueBoolSupplier;
import org.texastorque.torquelib.control.TorqueClickSupplier;
import org.texastorque.torquelib.control.TorqueRequestableTimeout;
import org.texastorque.torquelib.sensors.TorqueController;
import org.texastorque.torquelib.swerve.TorqueSwerveSpeeds;
import org.texastorque.torquelib.util.TorqueMath;

public final class Input extends TorqueInput<TorqueController> implements Subsystems {
    private static volatile Input instance;

    private final static double DEADBAND = 0.125;

    private final TorqueBoolSupplier resetGyro, speedDown, speedUp, goToIntake,
            runIntake, runOuttake, mid, high, wristUp, wristDown, wristRight, dunk,
            swapIntake, bucketAlign;

    private final TorqueRequestableTimeout driverRumbleTimeout, operatorRumbleTimeout;

    private Input() {
        driver = new TorqueController(0, DEADBAND);
        operator = new TorqueController(1, DEADBAND);

        resetGyro = new TorqueClickSupplier(driver::isRightCenterButtonDown);
        speedDown = new TorqueClickSupplier(driver::isLeftBumperDown);
        speedUp = new TorqueClickSupplier(driver::isRightBumperDown);
        bucketAlign = new TorqueBoolSupplier(driver::isAButtonDown);

        goToIntake = new TorqueClickSupplier(operator::isAButtonDown);
        runIntake = new TorqueBoolSupplier(operator::isRightTriggerDown);
        runOuttake = new TorqueBoolSupplier(operator::isLeftTriggerDown);
        mid = new TorqueClickSupplier(operator::isBButtonDown);
        high = new TorqueClickSupplier(operator::isYButtonDown);
        dunk = new TorqueClickSupplier(operator::isXButtonDown);
        wristUp = new TorqueBoolSupplier(operator::isDPADUpDown);
        wristDown = new TorqueClickSupplier(operator::isDPADDownDown);
        wristRight = new TorqueBoolSupplier(operator::isDPADRightDown);
        swapIntake = new TorqueClickSupplier(operator::isRightBumperDown);

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
        wristUp.onTrue(() -> wrist.setState(elevator.tooLow() ? Wrist.State.ROTATE_UP : Wrist.State.UP));
        wristRight.onTrue(() -> wrist.setState(elevator.tooLow() ? Wrist.State.ROTATE_RIGHT : Wrist.State.RIGHT));
        wristDown.onTrue(() -> wrist.setState(elevator.tooLow() ? Wrist.State.ROTATE_UP : Wrist.State.DOWN));
    }

    public void updateIntake() {
        runIntake.onTrue(() -> intake.setState(Intake.State.INTAKE));
        runOuttake.onTrue(() -> intake.setState(Intake.State.OUTTAKE));
    }

    public void updateElevator() {
        goToIntake.onTrue(() -> elevator.setState(Elevator.State.INTAKE));
        mid.onTrue(() -> elevator.setState(Elevator.State.MID));
        high.onTrue(() -> elevator.setState(Elevator.State.HIGH));
        dunk.onTrue(() -> elevator.setState(elevator.isAtMid() ? Elevator.State.MID_DUNK : Elevator.State.HIGH_DUNK));
        swapIntake.onTrue(() -> elevator.setState(Elevator.State.SWAP_INTAKE));
    }

    public void updateDrivebase() {
        resetGyro.onTrue(() -> drivebase.resetGyro());
        speedDown.onTrue(() -> drivebase.speedSetting.shiftDown());
        speedUp.onTrue(() -> drivebase.speedSetting.shiftUp());
        bucketAlign.onTrue(() -> drivebase.setState(Drivebase.State.BUCKET_ALIGN));

        final double xVelocity = TorqueMath.scaledLinearDeadband(driver.getLeftYAxis(), DEADBAND)
                * Drivebase.MAX_VELOCITY;
        final double yVelocity = TorqueMath.scaledLinearDeadband(driver.getLeftXAxis(), DEADBAND)
                * Drivebase.MAX_VELOCITY;
        final double rotationVelocity = TorqueMath.scaledLinearDeadband(driver.getRightXAxis(), DEADBAND)
                * Drivebase.MAX_ANGULAR_VELOCITY;

        drivebase.inputSpeeds = new TorqueSwerveSpeeds(xVelocity, yVelocity, rotationVelocity);
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
