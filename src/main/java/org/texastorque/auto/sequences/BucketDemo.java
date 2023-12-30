package org.texastorque.auto.sequences;

import java.util.function.BooleanSupplier;

import org.texastorque.Subsystems;
import org.texastorque.subsystems.*;
import org.texastorque.torquelib.auto.TorqueSequence;
import org.texastorque.torquelib.auto.commands.TorqueRun;
import org.texastorque.torquelib.auto.commands.TorqueSwitch;
import org.texastorque.torquelib.auto.commands.TorqueWaitTime;
import org.texastorque.torquelib.auto.commands.TorqueWaitUntil;
import org.texastorque.torquelib.auto.commands.TorqueWhile;

import edu.wpi.first.math.geometry.Rotation2d;

public class BucketDemo extends TorqueSequence implements Subsystems {

    public static class DumpBucket extends TorqueSequence {

        // Will probably need some going forward and stuff
        public DumpBucket() {
            addBlock(new TorqueRun(() -> elevator.setState(Elevator.State.THROW)));
            addBlock(new TorqueRun(() -> wrist.setState(Wrist.State.RIGHT)));
            addBlock(new TorqueWaitUntil(() -> wrist.isAtState()));
            addBlock(new TorqueWaitUntil(() -> elevator.isAtState()));
            

            addBlock(new TorqueRun(() -> drivebase.setInputSpeeds(-.4, 0, 0)));
            addBlock(new TorqueWaitTime(1));
            addBlock(new TorqueRun(() -> drivebase.setInputSpeeds(0, 0, 0)));

            addBlock(new TorqueRun(() -> intake.setState(Intake.State.OUTTAKE)));
            addBlock(new TorqueWaitTime(1));
            addBlock(new TorqueRun(() -> intake.setState(Intake.State.OFF)));
        }
    }

    public static class DriveToDumpSite extends TorqueSequence {

        private int tagToFind = 0;

        private boolean tagNotInView() {
            return drivebase.getTag(tagToFind).isEmpty();
        }

        private double timeToDeposit = 0;

        private boolean isCloseToTag() {
            var opt = drivebase.getTag(tagToFind);
            if (opt.isEmpty()) return false;
            return opt.get().distance <= 3.5;
        }

        private double tagAngle = 0;

        private double getTagAngle() {
            if (!tagNotInView())
                tagAngle = drivebase.getTag(tagToFind).get().angleOffset + drivebase.getGyro().getHeadingCW().getDegrees();
            return tagAngle;
        }

        public DriveToDumpSite(final BooleanSupplier isBlue, double timeToSpike) {

            addBlock(new TorqueRun(() -> tagToFind = isBlue.getAsBoolean() ? 2 : 4));

            addBlock(new TorqueWhile(this::tagNotInView, new TurnAndLook()));

            addBlock(new TorqueRun(() -> drivebase.setAlignTarget(this::getTagAngle)));

            addBlock(new TorqueRun(() -> drivebase.setInputSpeeds(-.5, 0, 0)));
            addBlock(new TorqueRun(() -> drivebase.setState(Drivebase.State.ALIGN_TO_ANGLE)));

            addBlock(new TorqueWaitUntil(this::isCloseToTag, (final double t) -> timeToDeposit = t));
            addBlock(new TorqueRun(() -> drivebase.setState(Drivebase.State.ROBOT_RELATIVE)));

            addBlock(new TorqueRun(() -> drivebase.setInputSpeeds(0, 0, 0)));

            addBlock(new DumpBucket().command());

            addBlock(new TorqueRun(() -> drivebase.setState(Drivebase.State.ROBOT_RELATIVE)));
            addBlock(new TorqueRun(() -> drivebase.setInputSpeeds(.5, 0, 0)));
            addBlock(new TorqueWaitTime(timeToSpike * .5));
            addBlock(new TorqueRun(() -> drivebase.setInputSpeeds(0, 0, 0)));

            addBlock(new TorqueRun(() -> wrist.setState(Wrist.State.UP)));
            addBlock(new TorqueWaitUntil(() -> wrist.isAtState()));

            addBlock(new TorqueRun(() -> elevator.setState(Elevator.State.INTAKE)));
            addBlock(new TorqueWaitUntil(() -> elevator.isAtState()));
        }
    }


    public static class AttackBucket extends TorqueSequence {

        private boolean isBucketBlue = false;

        private double timeToSpike = 0;

        public AttackBucket() {
            addBlock(new TorqueRun(() -> drivebase.setAlignTarget(drivebase.getDetectedBucket().get().getCenterX() 
                    + drivebase.getGyro().getHeadingCW().getDegrees())));

            addBlock(new TorqueRun(() -> isBucketBlue = drivebase.getDetectedBucket().get().isBlue()));

            addLog("Rotating Wrist");
            addBlock(new TorqueSwitch(drivebase::isBucketUpRight, 
                    new Wrist.AutoRotateWrist(Wrist.State.UP),
                    new Wrist.AutoRotateWrist(Wrist.State.RIGHT))
            );

            addBlock(new TorqueRun(() -> drivebase.setInputSpeeds(0, 0, 0))); 
            addBlock(new TorqueRun(() -> drivebase.setState(Drivebase.State.ALIGN_TO_ANGLE)));
            addBlock(new TorqueWaitUntil(() -> drivebase.isAligned()));

            addBlock(new TorqueRun(() -> drivebase.setState(Drivebase.State.ROBOT_RELATIVE)));
            addBlock(new TorqueRun(() -> intake.setState(Intake.State.INTAKE)));
            addBlock(new TorqueRun(() -> drivebase.setInputSpeeds(-.5, 0, 0)));

            addBlock(new TorqueWaitUntil(() -> intake.hasSpiked(), (final double t) -> timeToSpike = t));
            addBlock(new TorqueRun(() -> drivebase.setInputSpeeds(0, 0, 0)));
            addBlock(new TorqueRun(() -> elevator.setState(Elevator.State.SWAP_INTAKE)));
            addBlock(new TorqueWaitUntil(elevator::isAtState));

            addBlock(new DriveToDumpSite(() -> isBucketBlue, timeToSpike).command());

            // Drive back
            // addBlock(new TorqueRun(() -> drivebase.setState(Drivebase.State.ROBOT_RELATIVE)));
            // addBlock(new TorqueRun(() -> drivebase.setInputSpeeds(.5, 0, 0)));
            // addBlock(new TorqueWaitTime(() -> timeToSpike * .5));
            // addBlock(new TorqueRun(() -> drivebase.setInputSpeeds(0, 0, 0)));
        }
    }


    public static class TurnAndLook extends TorqueSequence {
        public TurnAndLook() {
            addBlock(new TorqueRun(() -> drivebase.setAlignTarget(
                    drivebase.getGyro().getHeadingCW().getDegrees() + 45)));
            addBlock(new TorqueRun(() -> drivebase.setInputSpeeds(0, 0, 0))); 
            addBlock(new TorqueRun(() -> drivebase.setState(Drivebase.State.ALIGN_TO_ANGLE)));
            addBlock(new TorqueWaitUntil(() -> drivebase.isAligned()));
            addBlock(new TorqueRun(() -> drivebase.setState(Drivebase.State.FIELD_RELATIVE)));
            addBlock(new TorqueRun(() -> drivebase.setInputSpeeds(0, 0, 0)));
            addBlock(new TorqueWaitTime(1));
        }
    }

    public static class BucketDemoLoop extends TorqueSequence {
        public BucketDemoLoop() {

            // if (bucket.isPresent())
            //    AttackBucket()
            // else
            //    FindBucket()
            addBlock(new TorqueSwitch(() -> drivebase.getDetectedBucket().isPresent(), new AttackBucket(), new TurnAndLook())); 
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
