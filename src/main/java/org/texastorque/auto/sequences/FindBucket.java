package org.texastorque.auto.sequences;

import java.util.Optional;
import org.texastorque.Subsystems;
import org.texastorque.subsystems.*;
import org.texastorque.subsystems.Wrist.AutoRotateWrist;
import org.texastorque.toast.lib.pipelines.ObjectDetector.DetectedObject;
import org.texastorque.torquelib.auto.TorqueSequence;
import org.texastorque.torquelib.auto.commands.TorqueRun;
import org.texastorque.torquelib.auto.commands.TorqueRunSequence;
import org.texastorque.torquelib.auto.commands.TorqueSequenceSwitch;
import org.texastorque.torquelib.auto.commands.TorqueWaitTime;
import org.texastorque.torquelib.auto.commands.TorqueWaitUntil;

public class FindBucket extends TorqueSequence implements Subsystems {

    // private DetectedObject bucket = ObjectDetector.emptyObject;
    private int bucketsFound = 0;

    public DetectedObject getBucket() {
        return drivebase.getDetectedBucket().get();
    }

    public FindBucket() {
        addBlock(new TorqueRunSequence(new AutoRotateWrist(Wrist.State.UP)));
        addBlock(new TorqueRun(() -> elevator.setState(Elevator.State.INTAKE)));
        addBlock(new TorqueRun(() -> drivebase.setState(Drivebase.State.ROBOT_RELATIVE)));

        addBlock(new TorqueRun(() -> drivebase.setInputSpeeds(0, 0, .75)));
        addBlock(new TorqueWaitUntil(() -> {
            final Optional<DetectedObject> opt = drivebase.getDetectedBucket();
            if (opt.isPresent()) {
                bucketsFound++;
            }
            if (bucketsFound >= 10) {
                // bucket = opt.get();
                return true;
            }
            return false;
        }));

        addBlock(new TorqueRun(() -> bucketsFound = 0));

        addBlock(new TorqueRun(() -> drivebase.setInputSpeeds(0, 0, 0)));
        addBlock(new TorqueWaitTime(2));

        addBlock(new TorqueSequenceSwitch(() -> drivebase.getDetectedBucket().isPresent(),
                new AttackBucket(this::getBucket)));
    }
}
