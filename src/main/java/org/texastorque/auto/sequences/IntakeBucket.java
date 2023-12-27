package org.texastorque.auto.sequences;

import org.texastorque.Subsystems;
import org.texastorque.subsystems.*;
import org.texastorque.torquelib.auto.TorqueSequence;
import org.texastorque.torquelib.auto.commands.TorqueRun;
import org.texastorque.torquelib.auto.commands.TorqueRunWhile;
import org.texastorque.torquelib.auto.commands.TorqueWaitTime;
import org.texastorque.torquelib.auto.commands.TorqueWaitUntil;

public class IntakeBucket extends TorqueSequence implements Subsystems {
    public IntakeBucket() {
        addBlock(new TorqueRunWhile(new TorqueRun(() -> drivebase.orientWristForBucket()),
                () -> !wrist.finishedFirstWristSequence));
        addBlock(new TorqueRun(() -> drivebase.setState(Drivebase.State.BUCKET_ALIGN)));
        addBlock(new TorqueWaitUntil(() -> drivebase.isAlignedToBucket()));
        addBlock(new TorqueRun(() -> drivebase.setState(Drivebase.State.ROBOT_RELATIVE)));
        addBlock(new TorqueRun(() -> intake.setState(Intake.State.INTAKE)));
        addBlock(new TorqueRun(() -> drivebase.setInputSpeeds(-.5, 0, 0)));
        addBlock(new TorqueWaitUntil(() -> intake.hasSpiked()));
        addBlock(new TorqueRun(() -> drivebase.setInputSpeeds(0, 0, 0)));
        addBlock(new TorqueRun(() -> elevator.setState(Elevator.State.MID)));
        addBlock(new TorqueWaitUntil(() -> elevator.isAtState()));
        addBlock(new TorqueRun(() -> wrist.setState(Wrist.State.DOWN)));
        addBlock(new TorqueWaitUntil(() -> wrist.isAtState()));
        addBlock(new TorqueWaitTime(1.5, () -> intake.setState(Intake.State.OUTTAKE)));
        // addBlock(new TorqueRun)
    }
}
