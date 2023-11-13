package org.texastorque.auto;

import java.util.HashMap;
import java.util.Map;
import org.texastorque.Subsystems;
import org.texastorque.torquelib.auto.TorqueCommand;
import org.texastorque.torquelib.auto.commands.TorqueRun;

public final class EventMap implements Subsystems {

    public static Map<String, TorqueCommand> get() {
        final Map<String, TorqueCommand> map = new HashMap<String, TorqueCommand>();

        map.put("high", new TorqueRun(() -> {
            arm.setState(Arm.State.INTAKE);
            // intake.setState(Intake.State.INTAKE);
        }));
        return map;
    }
}
