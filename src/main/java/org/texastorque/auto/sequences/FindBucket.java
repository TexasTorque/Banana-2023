package org.texastorque.auto.sequences;

import org.texastorque.Debug;
import org.texastorque.Subsystems;
import org.texastorque.subsystems.*;
import org.texastorque.subsystems.Wrist.AutoRotateWrist;
import org.texastorque.subsystems.Wrist.State;
import org.texastorque.torquelib.auto.TorqueSequence;
import org.texastorque.torquelib.auto.commands.TorqueRun;
import org.texastorque.torquelib.auto.commands.TorqueRunWhile;
import org.texastorque.torquelib.auto.commands.TorqueSequenceSwitch;
import org.texastorque.torquelib.auto.commands.TorqueWaitTime;
import org.texastorque.torquelib.auto.commands.TorqueWaitUntil;

public class FindBucket extends TorqueSequence implements Subsystems {
    public FindBucket() {
        addBlock(new TorqueRun(() -> wrist.setState(Wrist.State.UP)));
        addBlock(new TorqueRun(() -> elevator.setState(Elevator.State.INTAKE)));
        addBlock(new TorqueRun(() -> drivebase.setState(Drivebase.State.ROBOT_RELATIVE)));
        addBlock(new TorqueRun(() -> drivebase.setInputSpeeds(0, 0, .5)));
        addBlock(new TorqueWaitUntil(() -> drivebase.seesBucket()));
        addBlock(new TorqueRun(() -> drivebase.setInputSpeeds(0, 0, 0)));
        addBlock(new TorqueWaitTime(1));
        // addBlock(new TorqueRunWhile(new TorqueRun(() -> drivebase.orientWristForBucket()),
                // () -> !wrist.finishedRunningRotateSequence));
        // addBlock(new TorqueRunSequence(new AutoRotateWrist(State.UP)));

        addBlock(new TorqueSequenceSwitch(drivebase::isBucketUpRight, new AutoRotateWrist(State.UP), new AutoRotateWrist(State.RIGHT)));

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
        addBlock(new TorqueRun(() -> intake.setState(Intake.State.OUTTAKE)));
        addBlock(new TorqueWaitTime(1.5));
        addBlock(new TorqueRun(() -> intake.setState(Intake.State.OFF)));
        addBlock(new TorqueRun(() -> Debug.log("ENDED FIRST SEQUENCE", true)));
    }
}
