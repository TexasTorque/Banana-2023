package org.texastorque.auto.sequences;

import java.util.Map;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import org.texastorque.Field;
import org.texastorque.Subsystems;
import org.texastorque.subsystems.*;
import org.texastorque.torquelib.auto.TorqueSequence;
import org.texastorque.torquelib.auto.commands.TorqueFollowPath;
import org.texastorque.torquelib.auto.commands.TorqueRun;
import org.texastorque.torquelib.auto.commands.TorqueSwitch;
import org.texastorque.torquelib.auto.commands.TorqueWaitTime;
import org.texastorque.torquelib.auto.commands.TorqueWaitUntil;
import org.texastorque.torquelib.auto.commands.TorqueWhile;
import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class BucketDemo extends TorqueSequence implements Subsystems {

    public static class DumpBucket extends TorqueSequence {
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

            addBlock(new TorqueRun(() -> drivebase.setInputSpeeds(.4, 0, 0)));
            addBlock(new TorqueWaitTime(1));
            addBlock(new TorqueRun(() -> drivebase.setInputSpeeds(0, 0, 0)));

            addBlock(new TorqueRun(() -> elevator.setState(Elevator.State.INTAKE)));
            addBlock(new TorqueRun(() -> wrist.setState(Wrist.State.UP)));
        }
    }

    public static PathPlannerTrajectory generateTrajectoryToPose(final Pose2d targetPose) {
        final Pose2d currentPose = drivebase.getPose();

        final Rotation2d initialHeading = Rotation2d.fromDegrees(0);

        final PathPoint startingPoint = new PathPoint(currentPose.getTranslation(), initialHeading, currentPose.getRotation());
        final PathPoint midPoint = new PathPoint(new Translation2d(targetPose.getX(), targetPose.getY() - Math.signum(targetPose.getY() - currentPose.getY() * .5)), initialHeading, currentPose.getRotation()); // might have to change rotation to targetPose
        final PathPoint endingPoint = new PathPoint(targetPose.getTranslation(), initialHeading, targetPose.getRotation());

        final PathConstraints pathConstraints= new PathConstraints(.5, .5);

        return PathPlanner.generatePath(pathConstraints, startingPoint, midPoint, endingPoint);
    }

    public static class DriveToDumpSite extends TorqueSequence {

        private PathPlannerTrajectory generateTrajectory(final BooleanSupplier isBlue) {
            final Pose2d targetPose = isBlue.getAsBoolean() ? Field.BLUE_DUMP_ZONE : Field.ORANGE_DUMP_ZONE;
            return generateTrajectoryToPose(targetPose);
        }
 
        public DriveToDumpSite(final BooleanSupplier isBlue) {

            addBlock(new TorqueRun(() -> drivebase.setState(Drivebase.State.FIELD_RELATIVE)));
            addBlock(new TorqueFollowPath(drivebase, () -> generateTrajectory(isBlue), Map.of()));


            addBlock(new DumpBucket().command());

            addBlock(new TurnDegrees(() -> 180).command());
        }
    }


    public static class AttackBucket extends TorqueSequence {

        private boolean isBucketBlue = false;

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

            addBlock(new TorqueWaitUntil(() -> intake.hasSpiked()));
            addBlock(new TorqueWaitTime(.5));

            addBlock(new TorqueRun(() -> drivebase.setInputSpeeds(0, 0, 0)));
            addBlock(new TorqueRun(() -> elevator.setState(Elevator.State.SWAP_INTAKE)));
            addBlock(new TorqueWaitUntil(elevator::isAtState));

            addBlock(new DriveToDumpSite(() -> isBucketBlue).command());
        }
    }

    public static class TurnDegrees extends TorqueSequence {
        public TurnDegrees(final DoubleSupplier degrees) {
            addBlock(new TorqueRun(() -> drivebase.setAlignTarget(
                    drivebase.getGyro().getHeadingCW().getDegrees() + degrees.getAsDouble())));
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
            addBlock(new TorqueSwitch(() -> drivebase.getDetectedBucket().isPresent(), new AttackBucket(), new TurnDegrees(() -> 45))); 
        }
    }

    public BucketDemo() {
        addBlock(new Wrist.AutoRotateWrist(Wrist.State.UP).command());
        addBlock(new TorqueRun(() -> elevator.setState(Elevator.State.INTAKE)));
        addBlock(new TorqueWhile(() -> true, new BucketDemoLoop()));     
    }
}
