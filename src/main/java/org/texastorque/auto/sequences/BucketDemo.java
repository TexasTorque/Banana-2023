package org.texastorque.auto.sequences;

import org.texastorque.Subsystems;
import org.texastorque.auto.sequences.BucketDemo.AttackBucket;
import org.texastorque.subsystems.*;
import org.texastorque.torquelib.auto.TorqueSequence;
import org.texastorque.torquelib.auto.commands.TorqueRun;
import org.texastorque.torquelib.auto.commands.TorqueRunSequence;
import org.texastorque.torquelib.auto.commands.TorqueRunWhile;
import org.texastorque.torquelib.auto.commands.TorqueSwitch;
import org.texastorque.torquelib.auto.commands.TorqueWaitTime;
import org.texastorque.torquelib.auto.commands.TorqueWaitUntil;
import org.texastorque.torquelib.auto.commands.TorqueWhile;

import edu.wpi.first.math.geometry.Rotation2d;

public class BucketDemo extends TorqueSequence implements Subsystems {

    public static class DumpBucket extends TorqueSequence {

        private double angleToTurn = 0;

        public DumpBucket(final boolean isBlue) {
            addBlock(new TorqueRun(() -> angleToTurn = isBlue ? 90 : -90));
            addBlock(new TorqueRun(() -> drivebase.setAlignTarget(Rotation2d.fromDegrees(
                    drivebase.getGyro().getHeadingCW().getDegrees() + angleToTurn).getDegrees())));

            addBlock(new TorqueRun(() -> drivebase.setState(Drivebase.State.ALIGN_TO_ANGLE)));
            addBlock(new TorqueRun(() -> elevator.setState(Elevator.State.THROW)));

            addBlock(new TorqueWaitUntil(() -> drivebase.isAligned()));
            addBlock(new TorqueWaitUntil(() -> elevator.isAtState()));

            addBlock(new TorqueRun(() -> wrist.setState(Wrist.State.DOWN)));
            addBlock(new TorqueWaitUntil(() -> wrist.isAtState()));

            addBlock(new TorqueRun(() -> intake.setState(Intake.State.OUTTAKE)));
            addBlock(new TorqueWaitTime(1));
            addBlock(new TorqueRun(() -> intake.setState(Intake.State.OFF)));

            addBlock(new TorqueRun(() -> wrist.setState(Wrist.State.UP)));
            addBlock(new TorqueWaitUntil(() -> wrist.isAtState()));

            addBlock(new TorqueRun(() -> drivebase.setAlignTarget(Rotation2d.fromDegrees(drivebase.getGyro().getHeadingCW().getDegrees() - angleToTurn).getDegrees())));

            addBlock(new TorqueRun(() -> drivebase.setState(Drivebase.State.ALIGN_TO_ANGLE)));
            addBlock(new TorqueRun(() -> elevator.setState(Elevator.State.INTAKE)));

            addBlock(new TorqueWaitUntil(() -> drivebase.isAligned()));
            addBlock(new TorqueWaitUntil(() -> elevator.isAtState()));
        }
    }


    public static class FindBucket extends TorqueSequence {
        public FindBucket() {
            addBlock(new TorqueRun(() -> drivebase.setAlignTarget(Rotation2d.fromDegrees(
                    drivebase.getGyro().getHeadingCW().getDegrees() + 45).getDegrees())));
            addBlock(new TorqueRun(() -> drivebase.setState(Drivebase.State.ALIGN_TO_ANGLE)));
            addBlock(new TorqueWaitUntil(() -> drivebase.isAligned()));
            addBlock(new TorqueRun(() -> drivebase.setState(Drivebase.State.FIELD_RELATIVE)));
            addBlock(new TorqueRun(() -> drivebase.setInputSpeeds(0, 0, 0)));
            addBlock(new TorqueWaitTime(1));
        }
    }

    public static class AttackBucket extends TorqueSequence {

        private boolean isBucketBlue = false;

        public AttackBucket() {
            addBlock(new TorqueRun(() -> drivebase.setAlignTarget(drivebase.getDetectedBucket().get().getCenterX() 
                    + drivebase.getGyro().getHeadingCW().getDegrees())));

            addBlock(new TorqueRun(() -> isBucketBlue = drivebase.getDetectedBucket().get().isBlue()));

            addBlock(new TorqueSwitch(drivebase::isBucketUpRight, 
                    new Wrist.AutoRotateWrist(Wrist.State.UP),
                    new Wrist.AutoRotateWrist(Wrist.State.RIGHT))
            );

            addBlock(new TorqueRun(() -> drivebase.setState(Drivebase.State.ALIGN_TO_ANGLE)));
            addBlock(new TorqueWaitUntil(() -> drivebase.isAligned()));

            addBlock(new TorqueRun(() -> drivebase.setState(Drivebase.State.ROBOT_RELATIVE)));
            addBlock(new TorqueRun(() -> intake.setState(Intake.State.INTAKE)));
            addBlock(new TorqueRun(() -> drivebase.setInputSpeeds(-.5, 0, 0)));
            addBlock(new TorqueWaitUntil(() -> intake.hasSpiked()));
            addBlock(new TorqueRun(() -> drivebase.setInputSpeeds(0, 0, 0)));

            addBlock(new DumpBucket(isBucketBlue).command());
        }
    }


    public static class BucketDemoLoop extends TorqueSequence {
        public BucketDemoLoop() {

            // if (bucket.isPresent())
            //    AttackBucket()
            // else
            //    FindBucket()
            addBlock(new TorqueSwitch(() -> drivebase.getDetectedBucket().isPresent(), new AttackBucket(), new FindBucket())); 
        }
    }

    public BucketDemo() {
        addBlock(new Wrist.AutoRotateWrist(Wrist.State.UP).command());
        addBlock(new TorqueRun(() -> elevator.setState(Elevator.State.INTAKE)));

        // while (true)
        //    BucketDemoLoop();
        addBlock(new TorqueWhile(() -> true, new BucketDemoLoop()));     
    }
}
