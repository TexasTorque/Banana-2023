/**
 * Copyright 2023 Texas Torque.
 *
 * This file is part of Torque-2023, which is not licensed for distribution. For more details, see
 * ./license.txt or write <jus@justusl.com>.
 */
package org.texastorque.subsystems;

import org.texastorque.Debug;
import org.texastorque.Ports;
import org.texastorque.Subsystems;
import org.texastorque.toast.lib.Camera;
import org.texastorque.toast.lib.Toast;
import org.texastorque.toast.lib.pipelines.AprilTags;
import org.texastorque.toast.lib.pipelines.ObjectDetector;
import org.texastorque.torquelib.base.TorqueMode;
import org.texastorque.torquelib.base.TorqueState;
import org.texastorque.torquelib.base.TorqueStatorSubsystem;
import org.texastorque.torquelib.sensors.TorqueNavXGyro;
import org.texastorque.torquelib.swerve.TorqueSwerveSpeeds;
import org.texastorque.torquelib.swerve.TorqueSwerveX;
import edu.wpi.first.math.VecBuilder;
import org.texastorque.Field;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public final class Drivebase extends TorqueStatorSubsystem<Drivebase.State> implements Subsystems {
    public static enum State implements TorqueState {
        FIELD_RELATIVE(null), ROBOT_RELATIVE(null), XF(FIELD_RELATIVE), ZERO(
                FIELD_RELATIVE), BUCKET_ALIGN(ROBOT_RELATIVE);

        public final State parent;

        private State(final State parent) {
            this.parent = parent == null ? this : parent;
        }
    }

    public enum SpeedSetting {
        SLOW(.25), MID(.5), FAST(1.0);

        private static final SpeedSetting[] vals = values();

        public double speed;

        private SpeedSetting(final double speed) {
            this.speed = speed;
        }

        public SpeedSetting shiftUp() {
            return vals[Math.min((this.ordinal() + 1), vals.length - 2)];
        }

        public SpeedSetting shiftDown() {
            return vals[Math.max((this.ordinal() - 1), 0)];
        }
    }

    private static volatile Drivebase instance;

    public static final double WIDTH = Units.inchesToMeters(58 / 3);

    public static final Pose2d INITIAL_POS = new Pose2d(0, 0, Rotation2d.fromRadians(0));

    /**
     * Standard deviations of model states. Increase these numbers to trust your model's state
     * estimates less. This matrix is in the form [x, y, theta]ᵀ, with units in meters and radians,
     * then meters.
     */
    private static final Vector<N3> STATE_STDS =
            VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5));

    /**
     * Standard deviations of the vision measurements. Increase these numbers to trust global
     * measurements from vision less. This matrix is in the form [x, y, theta]ᵀ, with units in
     * meters and radians.
     */
    private static final Vector<N3> VISION_STDS =
            VecBuilder.fill(0.1, 0.1, Units.degreesToRadians(10));

    public final static double MAX_VELOCITY = 5, MAX_ANGULAR_VELOCITY = 8;

    public static SwerveModulePosition invertSwerveModuleDistance(final SwerveModulePosition pose) {
        return new SwerveModulePosition(-pose.distanceMeters, pose.angle);
    }

    public static synchronized final Drivebase getInstance() {
        return instance == null ? instance = new Drivebase() : instance;
    }

    private final Translation2d LOC_FL = new Translation2d(WIDTH / 2, WIDTH / 2),
            LOC_FR = new Translation2d(WIDTH / 2, -WIDTH / 2),
            LOC_BL = new Translation2d(-WIDTH / 2, WIDTH / 2),
            LOC_BR = new Translation2d(-WIDTH / 2, -WIDTH / 2);

    private final SwerveDriveKinematics kinematics;
    private final SwerveDrivePoseEstimator poseEstimator;

    public final Field2d fieldMap = new Field2d();

    private final TorqueSwerveX fl, fr, bl, br;

    private final TorqueNavXGyro gyro = TorqueNavXGyro.getInstance();

    private SwerveModuleState[] swerveStates;

    public TorqueSwerveSpeeds inputSpeeds = new TorqueSwerveSpeeds(0, 0, 0);

    public SpeedSetting speedSetting = SpeedSetting.FAST;

    public final double GYRO_COEFF = 0.2;

    public final Toast toast;

    public final PIDController bucketAlignPID;

    private Drivebase() {
        super(State.FIELD_RELATIVE);

        fl = new TorqueSwerveX("Front Left", Ports.FL_MOD, 0.1088);
        fr = new TorqueSwerveX("Front Right", Ports.FR_MOD, -2.01565);
        bl = new TorqueSwerveX("Back Left", Ports.BL_MOD, 0.6841);
        br = new TorqueSwerveX("Back Right", Ports.BR_MOD, -0.66280);


        kinematics = new SwerveDriveKinematics(LOC_FL, LOC_FR, LOC_BL, LOC_BR);

        poseEstimator = new SwerveDrivePoseEstimator(kinematics, gyro.getHeadingCCW(),
                getModulePositions(), INITIAL_POS, STATE_STDS, VISION_STDS);

        swerveStates = new SwerveModuleState[4];
        for (int i = 0; i < swerveStates.length; i++)
            swerveStates[i] = new SwerveModuleState();

        toast = new Toast(Field.getCurrentFieldLayout());

        toast.addCamera(new Camera("fl", Camera.transformInchDeg(0, 0, 0, 0)));

        toast.iterCams(cam -> cam.addPipeline(new ObjectDetector()));

        SmartDashboard.putData("FIELD", fieldMap);

        bucketAlignPID = new PIDController(10, 0, 0);
    }

    public boolean isState(State state) {
        return desiredState == state;
    }

    @Override
    public final void initialize(final TorqueMode mode) {
        mode.onAuto(() -> {
            desiredState = State.ROBOT_RELATIVE;
        });

        mode.onTeleop(() -> {
            desiredState = State.FIELD_RELATIVE;
        });
    }

    public SwerveModulePosition[] getModulePositions() {
        return new SwerveModulePosition[] {invertSwerveModuleDistance(fl.getPosition()),
                invertSwerveModuleDistance(fr.getPosition()),
                invertSwerveModuleDistance(bl.getPosition()),
                invertSwerveModuleDistance(br.getPosition())};
    }


    @Override
    public final void update(final TorqueMode mode) {
        updateFeedback();
        Debug.log("State", desiredState.toString());
        toast.update(null);

        double centerX =
                ((ObjectDetector) toast.getCamera("fl").getPipe(new ObjectDetector().getClass()))
                        .getBestObject().getCenterX();

        Debug.log("centerX", centerX);


        if (desiredState == State.XF) {
            manuallySetModuleStates(0.79, 2.36, 2.36, 0.79);
        } else if (desiredState == State.ZERO) {
            manuallySetModuleStates(0, 0, 0, 0);
        } else {
            if (mode.isTeleop()) {
                inputSpeeds = inputSpeeds
                        .times(!elevator.tooLow() ? SpeedSetting.SLOW.speed : speedSetting.speed);

                inputSpeeds = inputSpeeds.toFieldRelativeSpeeds(gyro.getHeadingCCW());
                // .plus(gyro.getAngularVelocity().times(GYRO_COEFF)));
            }

            if (desiredState == State.BUCKET_ALIGN) {
                // ObjectDetector objd = new ObjectDetector();
                // objd = (ObjectDetector) (toast.getCamera("fl").get().getPipe(new
                // ObjectDetector().getClass()));
                // double centerX = ((ObjectDetector) toast.getCamera("fl").get()
                // .getPipe(new ObjectDetector().getClass())).getBestObject().getCenterX();

                inputSpeeds.omegaRadiansPerSecond = -bucketAlignPID.calculate(centerX, 0);
                inputSpeeds.vxMetersPerSecond = 0;
                inputSpeeds.vyMetersPerSecond = 0;

            }

            swerveStates = kinematics.toSwerveModuleStates(inputSpeeds);

            SwerveDriveKinematics.desaturateWheelSpeeds(swerveStates, MAX_VELOCITY);

            if (inputSpeeds.hasZeroVelocity()) {
                manuallySetModuleStates(swerveStates[0].angle.getRadians(),
                        swerveStates[1].angle.getRadians(), swerveStates[2].angle.getRadians(),
                        swerveStates[3].angle.getRadians());

            } else {
                fl.setDesiredState(swerveStates[0]);
                fr.setDesiredState(swerveStates[1]);
                bl.setDesiredState(swerveStates[2]);
                br.setDesiredState(swerveStates[3]);
            }
        }

        desiredState = desiredState.parent;

        Debug.log("Speed Shift State", speedSetting.toString());
        Debug.log("gyro", gyro.getHeadingCCW().getDegrees());
    }

    public void resetGyro() {
        gyro.setOffsetCW(Rotation2d.fromRadians(0));
        poseEstimator.resetPosition(gyro.getHeadingCCW(), getModulePositions(), INITIAL_POS);
    }

    private void updateFeedback() {
        poseEstimator.update(gyro.getHeadingCCW(), getModulePositions());

        fieldMap.setRobotPose(poseEstimator.getEstimatedPosition());

        Debug.log("Current Robot Pose", poseEstimator.getEstimatedPosition().toString());
    }

    private void manuallySetModuleStates(final double flAngle, final double frAngle,
            final double blAngle, final double brAngle) {
        fl.setDesiredState(new SwerveModuleState(0, Rotation2d.fromRadians(flAngle)));
        fr.setDesiredState(new SwerveModuleState(0, Rotation2d.fromRadians(frAngle)));
        bl.setDesiredState(new SwerveModuleState(0, Rotation2d.fromRadians(blAngle)));
        br.setDesiredState(new SwerveModuleState(0, Rotation2d.fromRadians(brAngle)));
    }
}
