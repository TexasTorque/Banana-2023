package org.texastorque;

import org.texastorque.subsystems.*;

public interface Subsystems {
    public final Drivebase drivebase = Drivebase.getInstance();
    public final Elevator elevator = Elevator.getInstance();
    public final Wrist wrist = Wrist.getInstance();
    public final Intake intake = Intake.getInstance();
    public final Lights lights = Lights.getInstance();
}
