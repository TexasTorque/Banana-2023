package org.texastorque;

import org.texastorque.subsystems.Drivebase;
import org.texastorque.torquelib.base.TorqueInput;
import org.texastorque.torquelib.control.TorqueBoolSupplier;
import org.texastorque.torquelib.control.TorqueClickSupplier;
import org.texastorque.torquelib.sensors.TorqueController;
import org.texastorque.torquelib.util.TorqueMath;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public final class Input extends TorqueInput<TorqueController> implements Subsystems {
    private static volatile Input instance;

    private final static double DEADBAND = 0.125;

    private final TorqueBoolSupplier resetGyro, speedDown, speedUp, useRotationLock;

    private Input() {
        driver = new TorqueController(0, 0.1);
        operator = new TorqueController(1, 0.1);

        resetGyro = new TorqueClickSupplier(driver::isRightCenterButtonDown);
        speedDown = new TorqueClickSupplier(driver::isLeftBumperDown);
        speedUp = new TorqueClickSupplier(driver::isRightBumperDown);
        useRotationLock = new TorqueBoolSupplier(driver::isAButtonDown);
    }

    @Override
    public final void update() {
        updateDrivebase();
    }

    public void updateDrivebase() {
        resetGyro.onTrue(() -> drivebase.resetGyro());
        speedDown.onTrue(() -> drivebase.speedSetting.shiftDown());
        speedUp.onTrue(() -> drivebase.speedSetting.shiftUp());
        drivebase.useRotationLock = useRotationLock.get();

        final double xVelocity = TorqueMath.scaledLinearDeadband(driver.getLeftYAxis(), DEADBAND)
                * Drivebase.MAX_VELOCITY_TELEOP;
        final double yVelocity = TorqueMath.scaledLinearDeadband(driver.getLeftXAxis(), DEADBAND)
                * Drivebase.MAX_VELOCITY_TELEOP;
        final double rotationVelocity =
                TorqueMath.scaledLinearDeadband(driver.getRightXAxis(), DEADBAND)
                        * Drivebase.MAX_ANGULAR_VELOCITY;

        drivebase.setInputSpeeds(xVelocity, yVelocity, rotationVelocity);
        SmartDashboard.putBoolean("rotationLock", useRotationLock.get());
    }

    public static final synchronized Input getInstance() {
        return instance == null ? instance = new Input() : instance;
    }
}
