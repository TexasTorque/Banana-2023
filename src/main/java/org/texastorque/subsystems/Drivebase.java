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
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public final class Drivebase extends TorqueStatorSubsystem<Drivebase.State> implements Subsystems {
    public static enum State implements TorqueState {
        FIELD_RELATIVE(null), ROBOT_RELATIVE(null), XF(FIELD_RELATIVE), ZERO(FIELD_RELATIVE);

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

    private SwerveModuleState[] swerveStates;

    public TorqueSwerveSpeeds inputSpeeds;

    public SpeedSetting speedSetting = SpeedSetting.FAST;

    public double ANGULAR_VELOCITY_COEFFICIENT = .085;

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
        return new SwerveModulePosition[] {fl.getPosition(), fr.getPosition(), bl.getPosition(),
                br.getPosition()};
    }

    public void setInputSpeeds(final double xVelocity, final double yVelocity,
            final double rVelocity) {
        inputSpeeds = new TorqueSwerveSpeeds(xVelocity, yVelocity, rVelocity);
    }

    public boolean rotationLock = true;

    @Override
    public final void update(final TorqueMode mode) {
        updateFeedback();
        Debug.log("State", desiredState.toString());

        ANGULAR_VELOCITY_COEFFICIENT =
                SmartDashboard.getNumber("Angular Velocity Coeff", ANGULAR_VELOCITY_COEFFICIENT);
        SmartDashboard.putNumber("Gyro Angular Velocity", gyro.getAngularVelocity().getDegrees());


        if (desiredState == State.XF) {
            manuallySetModuleStates(0.79, 2.36, 2.36, 0.79);
        } else if (desiredState == State.ZERO) {
            manuallySetModuleStates(0, 0, 0, 0);
        } else {
            if (mode.isTeleop()) {
                inputSpeeds = inputSpeeds
                        .toFieldRelativeSpeeds(gyro.getHeadingCW()
                                .plus(rotationLock
                                        ? gyro.getAngularVelocity()
                                                .times(ANGULAR_VELOCITY_COEFFICIENT)
                                        : new Rotation2d(0)))
                        .times(elevator.isAtStow() ? speedSetting.speed : SpeedSetting.SLOW.speed);
            }

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

        desiredState = desiredState.parent;

        Debug.log("Speed Shift State", speedSetting.toString());
    }

    public void resetGyro() {
        gyro.setOffsetCW(Rotation2d.fromRadians(0));
        poseEstimator.resetPosition(gyro.getHeadingCW(), getModulePositions(), INITIAL_POS);
    }

    public void resetPose(final Pose2d pose) {
        poseEstimator.resetPosition(gyro.getHeadingCW(), getModulePositions(), pose);
    }

    public Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
    }

    private void updateFeedback() {
        poseEstimator.update(gyro.getHeadingCW(), getModulePositions());

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
}
