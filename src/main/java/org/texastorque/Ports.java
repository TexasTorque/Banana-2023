package org.texastorque;

import org.texastorque.torquelib.swerve.base.TorqueSwerveModule.SwervePorts;

public final class Ports {
    public static final SwervePorts FL_MOD = new SwervePorts(2, 1, 11);
    public static final SwervePorts FR_MOD = new SwervePorts(4, 3, 9);
    public static final SwervePorts BR_MOD = new SwervePorts(6, 5, 10);
    public static final SwervePorts BL_MOD = new SwervePorts(8, 7, 12);

    public static final int ELEVATOR = 13;
    public static final int WRIST = 14;
    public static final int INTAKE_ROLLERS = 15;
}
