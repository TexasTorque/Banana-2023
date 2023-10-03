/**
 * Copyright 2023 Texas Torque.
 *
 * This file is part of Torque-2023, which is not licensed for distribution. For more details, see
 * ./license.txt or write <jus@justusl.com>.
 */
package org.texastorque.subsystems;

import org.texastorque.Debug;
import org.texastorque.Field;
import org.texastorque.Ports;
import org.texastorque.Subsystems;
import org.texastorque.torquelib.base.TorqueMode;
import org.texastorque.torquelib.base.TorqueState;
import org.texastorque.torquelib.base.TorqueStatorSubsystem;
import org.texastorque.torquelib.sensors.TorqueNavXGyro;
import org.texastorque.torquelib.swerve.TorqueSwerveSpeeds;
import org.texastorque.torquelib.swerve.TorqueSwerveX;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
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
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public final class Drivebase extends TorqueStatorSubsystem<Drivebase.State> implements Subsystems {
    public static enum State implements TorqueState {
        FIELD_RELATIVE(null), ROBOT_RELATIVE(null), XF(FIELD_RELATIVE);

        public final State parent;

        private State(final State parent) {
            this.parent = parent == null ? this : parent;
        }
    }

    public enum SpeedSetting {
        SLOW(.25), MID(.5), FAST(1.0), SEQ(1);

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

    public static class SpeedSequence {
        final double initSpeed, finalSpeed, duration, startTime, speedDeceleration;

        // Linearly decreases the speed every second for a duration of time
        public SpeedSequence(final SpeedSetting initSpeed, final SpeedSetting finalSpeed,
                final double duration) {
            this.initSpeed = initSpeed.speed;
            this.finalSpeed = finalSpeed.speed;
            this.duration = duration;
            speedDeceleration = (this.initSpeed - this.finalSpeed) / duration;
            startTime = Timer.getFPGATimestamp();
        }

        public double get() {
            return Math.max(initSpeed - speedDeceleration * (Timer.getFPGATimestamp() - startTime),
                    finalSpeed);
        }
    }

    private static volatile Drivebase instance;

    public static final double WIDTH = Units.inchesToMeters(18), LENGTH = Units.inchesToMeters(21);

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

    public final static double MAX_VELOCITY = 3.5, MAX_ACCELERATION = 3.5,
            MAX_ANGULAR_VELOCITY = 2 * Math.PI, MAX_ANGULAR_ACCELERATION = 2 * Math.PI;

    public static SwerveModulePosition invertSwerveModuleDistance(final SwerveModulePosition pose) {
        return new SwerveModulePosition(-pose.distanceMeters, pose.angle);
    }

    public static synchronized final Drivebase getInstance() {
        return instance == null ? instance = new Drivebase() : instance;
    }

    private final Translation2d LOC_FL = new Translation2d(LENGTH / 2, -WIDTH / 2),
            LOC_FR = new Translation2d(LENGTH / 2, WIDTH / 2),
            LOC_BL = new Translation2d(-LENGTH / 2, -WIDTH / 2),
            LOC_BR = new Translation2d(-LENGTH / 2, WIDTH / 2);

    private final SwerveDriveKinematics kinematics;
    private final SwerveDrivePoseEstimator poseEstimator;

    public final Field2d fieldMap = new Field2d();

    private final TorqueSwerveX fl, fr, bl, br;

    private final TorqueNavXGyro gyro = TorqueNavXGyro.getInstance();

    private SwerveModuleState[] swerveStates;

    public TorqueSwerveSpeeds inputSpeeds = new TorqueSwerveSpeeds(0, 0, 0);

    public boolean isRotationLocked = true;

    public SpeedSetting speedSetting = SpeedSetting.FAST;

    public SpeedSequence speedSequence = new SpeedSequence(speedSetting, speedSetting, -1);

    private Drivebase() {
        super(State.FIELD_RELATIVE);

        fl = new TorqueSwerveX("Front Left", Ports.FL_MOD, 0);
        fr = new TorqueSwerveX("Front Right", Ports.FR_MOD, 0);
        bl = new TorqueSwerveX("Back Left", Ports.BL_MOD, 0);
        br = new TorqueSwerveX("Back Right", Ports.BR_MOD, 0);

        kinematics = new SwerveDriveKinematics(LOC_BL, LOC_BR, LOC_FL, LOC_FR);

        poseEstimator = new SwerveDrivePoseEstimator(kinematics, gyro.getHeadingCCW(),
                getModulePositions(), INITIAL_POS, STATE_STDS, VISION_STDS);

        swerveStates = new SwerveModuleState[4];
        for (int i = 0; i < swerveStates.length; i++)
            swerveStates[i] = new SwerveModuleState();

        SmartDashboard.putData("FIELD", fieldMap);
    }

    public boolean isState(State state) {
        return desiredState == state;
    }

    @Override
    public final void initialize(final TorqueMode mode) {

        mode.onAuto(() -> {
            isRotationLocked = false;
            desiredState = State.ROBOT_RELATIVE;
        });

        mode.onTeleop(() -> {
            isRotationLocked = true;
            desiredState = State.FIELD_RELATIVE;
        });
    }

    public SwerveModulePosition[] getModulePositions() {
        return new SwerveModulePosition[] {invertSwerveModuleDistance(fl.getPosition()),
                invertSwerveModuleDistance(fr.getPosition()),
                invertSwerveModuleDistance(bl.getPosition()),
                invertSwerveModuleDistance(br.getPosition())};
    }

    public void convertToFieldRelative() {
        inputSpeeds = inputSpeeds.toFieldRelativeSpeeds(gyro.getHeadingCCW());
    }

    @Override
    public final void update(final TorqueMode mode) {
        updateFeedback();


        if (desiredState == State.XF) {
            xFactor();
        } else {
            if (mode.isTeleop()) {
                inputSpeeds =
                        inputSpeeds.times(speedSetting == SpeedSetting.SEQ ? speedSequence.get()
                                : speedSetting.speed);

                convertToFieldRelative();
            }

            swerveStates = kinematics.toSwerveModuleStates(inputSpeeds);

            SwerveDriveKinematics.desaturateWheelSpeeds(swerveStates, MAX_VELOCITY);

            if (inputSpeeds.hasZeroVelocity()) {
                preseveModulePositions();
            } else {
                fl.setDesiredState(swerveStates[0]);
                fr.setDesiredState(swerveStates[1]);
                bl.setDesiredState(swerveStates[2]);
                br.setDesiredState(swerveStates[3]);
            }
        }

        desiredState = desiredState.parent;
        Debug.log("Speed Shift State", speedSetting.toString());
        Debug.log("Speed Shift Value",
                speedSetting == SpeedSetting.SEQ ? speedSequence.get() : speedSetting.speed);
    }

    public void resetGyro() {
        gyro.setOffsetCW(Rotation2d.fromRadians(0));
    }

    private void updateFeedback() {
        poseEstimator.update(gyro.getHeadingCCW(), getModulePositions());

        fieldMap.setRobotPose(DriverStation.getAlliance() == DriverStation.Alliance.Blue
                ? poseEstimator.getEstimatedPosition()
                : Field.reflectPosition(poseEstimator.getEstimatedPosition()));

        Debug.log("Current Robot Pose", poseEstimator.getEstimatedPosition().toString());
    }

    private void preseveModulePositions() {
        fl.setDesiredState(new SwerveModuleState(0, swerveStates[0].angle));
        fr.setDesiredState(new SwerveModuleState(0, swerveStates[1].angle));
        bl.setDesiredState(new SwerveModuleState(0, swerveStates[2].angle));
        br.setDesiredState(new SwerveModuleState(0, swerveStates[3].angle));
    }

    private void xFactor() {
        fl.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
        fr.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(135)));
        bl.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(135)));
        br.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    }
}
