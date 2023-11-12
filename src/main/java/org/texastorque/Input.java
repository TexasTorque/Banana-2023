package org.texastorque;

import org.texastorque.subsystems.Drivebase;
import org.texastorque.torquelib.base.TorqueInput;
import org.texastorque.torquelib.control.TorqueBoolSupplier;
import org.texastorque.torquelib.control.TorqueToggleSupplier;
import org.texastorque.torquelib.sensors.TorqueController;
import org.texastorque.torquelib.swerve.TorqueSwerveSpeeds;
import org.texastorque.torquelib.util.TorqueMath;

public final class Input extends TorqueInput<TorqueController> implements Subsystems {
    private static volatile Input instance;

    private final static double DEADBAND = 0.125;

    private final TorqueBoolSupplier zero, xf;

    private Input() {
        driver = new TorqueController(0, 0.1);
        operator = new TorqueController(1, 0.1);

        zero = new TorqueToggleSupplier(driver::isRightCenterButtonDown);
        xf = new TorqueToggleSupplier(driver::isXButtonDown);
    }

    @Override
    public final void update() {
        updateDrivebase();
    }

    public void updateDrivebase() {
        zero.onTrue(() -> drivebase.setState(Drivebase.State.ZERO));
        xf.onTrue(() -> drivebase.setState(Drivebase.State.XF));

        final double xVelocity = TorqueMath.scaledLinearDeadband(driver.getLeftYAxis(), DEADBAND)
                * Drivebase.MAX_VELOCITY;
        final double yVelocity = TorqueMath.scaledLinearDeadband(driver.getLeftXAxis(), DEADBAND)
                * Drivebase.MAX_VELOCITY;


        final double rotationVelocity =
                TorqueMath.scaledLinearDeadband(-driver.getRightXAxis(), DEADBAND)
                        * Drivebase.MAX_ANGULAR_VELOCITY;
        drivebase.inputSpeeds = new TorqueSwerveSpeeds(xVelocity, yVelocity, rotationVelocity);

    }

    public static final synchronized Input getInstance() {
        return instance == null ? instance = new Input() : instance;
    }
}
