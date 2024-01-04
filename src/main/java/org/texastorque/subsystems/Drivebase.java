/**
 * Copyright 2023 Texas Torque.
 *
 * This file is part of Torque-2023, which is not licensed for distribution. For more details, see
 * ./license.txt or write <jus@justusl.com>.
 */
package org.texastorque.subsystems;

import java.util.List;
import java.util.Optional;
import java.util.function.DoubleSupplier;

import javax.swing.text.html.Option;

import org.texastorque.Debug;
import org.texastorque.Field;
import org.texastorque.Ports;
import org.texastorque.Subsystems;
import org.texastorque.toast.lib.Camera;
import org.texastorque.toast.lib.Toast;
import org.texastorque.toast.lib.pipelines.AprilTags;
import org.texastorque.toast.lib.pipelines.BucketDetector;
import org.texastorque.toast.lib.pipelines.AprilTags.AprilTag;
import org.texastorque.torquelib.auto.commands.TorqueFollowPath.TorquePathingDrivebase;
import org.texastorque.torquelib.base.TorqueMode;
import org.texastorque.torquelib.base.TorqueState;
import org.texastorque.torquelib.base.TorqueStatorSubsystem;
import org.texastorque.torquelib.sensors.TorqueNavXGyro;
import org.texastorque.torquelib.swerve.TorqueSwerveSpeeds;
import org.texastorque.torquelib.swerve.TorqueSwerveX;
import org.texastorque.torquelib.util.TorqueMath;
import edu.wpi.first.math.VecBuilder;
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
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public final class Drivebase extends TorqueStatorSubsystem<Drivebase.State> implements Subsystems, TorquePathingDrivebase {
    public static enum State implements TorqueState {
        FIELD_RELATIVE(null), ROBOT_RELATIVE(null), ALIGN_TO_ANGLE(ROBOT_RELATIVE), XF(
                FIELD_RELATIVE), ZERO(FIELD_RELATIVE);

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

    public final static double MAX_VELOCITY_TELEOP = 4.6, MAX_ACCELERATION = 2,
            MAX_ANGULAR_VELOCITY = 6;

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

    public TorqueNavXGyro getGyro() {
        return gyro;
    }

    private SwerveModuleState[] swerveStates;

    public final Toast toast;

    public TorqueSwerveSpeeds inputSpeeds;

    public SpeedSetting speedSetting = SpeedSetting.FAST;

    public double ANGULAR_VELOCITY_COEFFICIENT = .085;

    private final PIDController alignPID;

    public Optional<BucketDetector.BucketDetection> getDetectedBucket() {
        return ((BucketDetector) toast.getCamera("tower").getPipe(new BucketDetector().getClass()))
                .getBestObject();
    }

    public List<AprilTag> getAprilTags() {
        return ((AprilTags) toast.getCamera("base").getPipe(new AprilTags().getClass())).getDetections();
    }

    public Optional<AprilTag> getTag(final int id) {
        for (final AprilTag tag : getAprilTags()) {
            if (tag.id == id) {
                return Optional.of(tag);
            }
        }
        return Optional.empty();
    }

    private Drivebase() {
        super(State.FIELD_RELATIVE);

        fl = new TorqueSwerveX("Front Left", Ports.FL_MOD, 0.1088);
        fr = new TorqueSwerveX("Front Right", Ports.FR_MOD, -2.01565);
        bl = new TorqueSwerveX("Back Left", Ports.BL_MOD, 0.6841);
        br = new TorqueSwerveX("Back Right", Ports.BR_MOD, -0.66280);

        inputSpeeds = new TorqueSwerveSpeeds(0, 0, 0);

        kinematics = new SwerveDriveKinematics(LOC_FL, LOC_FR, LOC_BL, LOC_BR);

        poseEstimator = new SwerveDrivePoseEstimator(kinematics, gyro.getHeadingCW(),
                getModulePositions(), INITIAL_POS, STATE_STDS, VISION_STDS);

        swerveStates = new SwerveModuleState[4];
        for (int i = 0; i < swerveStates.length; i++)
            swerveStates[i] = new SwerveModuleState();

        SmartDashboard.putData("FIELD", fieldMap);
        SmartDashboard.putNumber("Angular Velocity Coeff", ANGULAR_VELOCITY_COEFFICIENT);

        toast = new Toast(Field.getCurrentFieldLayout());

        alignPID = new PIDController(.15, 0, 0);
        alignPID.enableContinuousInput(0, 360);

        final double a = 8.258;
        final double b = 10.52;
        final double h = 9.446;
        final double p = 51;
        
        toast.addCamera(new Camera("tower", Camera.transformInchDeg(b, a, h, 0, p, 0)));
        toast.getCamera("tower").addPipeline(new BucketDetector());

        toast.addCamera(new Camera("base", Camera.transformInchDeg(b, -a, h, 0, p, 0)));
        toast.getCamera("base").addPipeline(new AprilTags());
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

    public void setInputSpeeds(final double xVelocity, final double yVelocity,
            final double rVelocity) {
        inputSpeeds = new TorqueSwerveSpeeds(xVelocity, yVelocity, rVelocity);
    }

    private DoubleSupplier alignTarget = () -> 0;

    public void setAlignTarget(final double target) {
        alignTarget = () -> target;
    }

    public void setAlignTarget(final DoubleSupplier target) {
        alignTarget = target;
    }

    private double getAlignTarget() {
        return TorqueMath.constrain0to360(alignTarget.getAsDouble());
    }

    public boolean isAligned() {
        return TorqueMath.toleranced(gyro.getHeadingCW().getDegrees(), 
                getAlignTarget(), 5); // this constrains [0°, 360°)
    }

    @Override
    public final void update(final TorqueMode mode) {
        updateFeedback();
        Debug.log("State", desiredState.toString());
        Debug.log("Aligned To Bucket", isAligned());

        if (desiredState == State.XF) {
            manuallySetModuleStates(0.79, 2.36, 2.36, 0.79);
        } else if (desiredState == State.ZERO) {
            manuallySetModuleStates(0, 0, 0, 0);
        } else {
            if (mode.isTeleop()) {
                inputSpeeds = inputSpeeds.toFieldRelativeSpeeds(gyro.getHeadingCW().times(-1))
                        .times(elevator.isLowCG() ? speedSetting.speed : SpeedSetting.SLOW.speed);
            }

            if (desiredState == State.ALIGN_TO_ANGLE) {
                inputSpeeds.omegaRadiansPerSecond = TorqueMath.constrain(alignPID.calculate(gyro.getHeadingCW().getDegrees(), alignTarget.getAsDouble()), .75);
            }

            Debug.log("Align To Target Goal", alignTarget.getAsDouble());
            Debug.log("Gyro", gyro.getHeadingCW().getDegrees());
            Debug.log("seesBucket", getDetectedBucket().isPresent());

            swerveStates = kinematics.toSwerveModuleStates(inputSpeeds);

            SwerveDriveKinematics.desaturateWheelSpeeds(swerveStates, MAX_VELOCITY_TELEOP);

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

        if (mode.isTeleop())
            desiredState = desiredState.parent;

        Debug.log("Speed Shift State", speedSetting.toString());

        if (getTag(2).isPresent()) {
            Debug.log("April Tag 2 Align Distance", getTag(2).get().distance);
        }
    }

    public boolean isBucketUpRight() {
        final Optional<BucketDetector.BucketDetection> detectedBucket = getDetectedBucket();
        if (detectedBucket.isEmpty())
            return true;
        return detectedBucket.get().isUpRight();
    }

    public void resetGyro() {
        gyro.setOffsetCW(Rotation2d.fromRadians(0));
        poseEstimator.resetPosition(gyro.getHeadingCW(), getModulePositions(), INITIAL_POS);
    }

    public double getGyroAngle() {
        return gyro.getHeadingCW().getDegrees();
    }

    public void resetPose(final Pose2d pose) {
        poseEstimator.resetPosition(gyro.getHeadingCW(), getModulePositions(), pose);
    }

    public Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
    }

    private void updateFeedback() {
        // toast.update(poseEstimator::addVisionMeasurement);
        toast.update((Pose2d p, Double d) -> {}); // do nothing w/ vision

        poseEstimator.update(gyro.getHeadingCCW(), getModulePositions());

        fieldMap.setRobotPose(poseEstimator.getEstimatedPosition());

        fieldMap.setRobotPose(DriverStation.getAlliance() == DriverStation.Alliance.Blue
                ? poseEstimator.getEstimatedPosition()
                : Field.reflectPosition(poseEstimator.getEstimatedPosition()));

        Debug.log("Current Robot Pose", poseEstimator.getEstimatedPosition().toString());
    }

    private void manuallySetModuleStates(final double flAngle, final double frAngle,
            final double blAngle, final double brAngle) {
        fl.setDesiredState(new SwerveModuleState(0, Rotation2d.fromRadians(flAngle)));
        fr.setDesiredState(new SwerveModuleState(0, Rotation2d.fromRadians(frAngle)));
        bl.setDesiredState(new SwerveModuleState(0, Rotation2d.fromRadians(blAngle)));
        br.setDesiredState(new SwerveModuleState(0, Rotation2d.fromRadians(brAngle)));
    }

    @Override
    public void setPose(final Pose2d pose) {
        resetPose(pose);
    }

    @Override
    public void setInputSpeeds(final TorqueSwerveSpeeds speeds) {
        this.inputSpeeds = speeds.times(-1, -1, -1);
    }

}
