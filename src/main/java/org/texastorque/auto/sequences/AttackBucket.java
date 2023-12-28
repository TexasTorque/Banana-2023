package org.texastorque.auto.sequences;

import java.util.function.Supplier;
import org.texastorque.Subsystems;
import org.texastorque.subsystems.*;
import org.texastorque.subsystems.Wrist.AutoRotateWrist;
import org.texastorque.subsystems.Wrist.State;
import org.texastorque.toast.lib.pipelines.ObjectDetector.DetectedObject;
import org.texastorque.torquelib.auto.TorqueSequence;
import org.texastorque.torquelib.auto.commands.TorqueRun;
import org.texastorque.torquelib.auto.commands.TorqueSequenceSwitch;
import org.texastorque.torquelib.auto.commands.TorqueWaitTime;
import org.texastorque.torquelib.auto.commands.TorqueWaitUntil;

public class AttackBucket extends TorqueSequence implements Subsystems {
    public AttackBucket(final Supplier<DetectedObject> bucket) {
        addBlock(new TorqueRun(() -> drivebase
                .setAlignTarget(bucket.get().getCenterX() + drivebase.getGyroAngle())));

        addBlock(new TorqueSequenceSwitch(drivebase::isBucketUpRight, new AutoRotateWrist(State.UP),
                new AutoRotateWrist(State.RIGHT)));

        addBlock(new TorqueRun(() -> drivebase.setState(Drivebase.State.ALIGN_TO_ANGLE)));
        addBlock(new TorqueWaitUntil(() -> drivebase.isAligned()));

        addBlock(new TorqueRun(() -> drivebase.setState(Drivebase.State.ROBOT_RELATIVE)));
        addBlock(new TorqueRun(() -> intake.setState(Intake.State.INTAKE)));
        addBlock(new TorqueRun(() -> drivebase.setInputSpeeds(-.5, 0, 0)));
        addBlock(new TorqueWaitUntil(() -> intake.hasSpiked()));
        addBlock(new TorqueRun(() -> drivebase.setInputSpeeds(0, 0, 0)));

        addBlock(new TorqueRun(() -> drivebase.setState(Drivebase.State.ROBOT_RELATIVE)));
        addBlock(new TorqueRun(() -> drivebase.setInputSpeeds(0, 0, -1)));
        addBlock(new TorqueRun(() -> elevator.setState(Elevator.State.THROW)));
        addBlock(new TorqueWaitTime(.5));
        addBlock(new TorqueRun(() -> drivebase.setInputSpeeds(0, 0, 0)));
        addBlock(new TorqueWaitUntil(() -> elevator.isAtState()));
        addBlock(new TorqueRun(() -> wrist.setState(Wrist.State.DOWN)));
        addBlock(new TorqueWaitUntil(() -> wrist.isAtState()));

        addBlock(new TorqueRun(() -> intake.setState(Intake.State.OUTTAKE)));
        addBlock(new TorqueWaitTime(1.5));
        addBlock(new TorqueRun(() -> intake.setState(Intake.State.OFF)));
        addBlock(new TorqueRun(() -> drivebase.setInputSpeeds(0, 0, 1)));
        addBlock(new TorqueWaitTime(1.5));

        addBlock(new TorqueRun(() -> drivebase.setInputSpeeds(.75, 0, 0)));
        addBlock(new TorqueWaitTime(1.5));
        addBlock(new TorqueRun(() -> drivebase.setInputSpeeds(0, 0, 0)));
    }
}
