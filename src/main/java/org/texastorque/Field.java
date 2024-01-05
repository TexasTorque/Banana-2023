/**
 * Copyright 2023 Texas Torque.
 *
 * This file is part of Torque-2023, which is not licensed for distribution.
 * For more details, see ./license.txt or write <jus@justusl.com>.
 */
package org.texastorque;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
public final class Field {
   
    public static final double FIELD_LENGTH = Units.inchesToMeters(651.25);
    public static final double FIELD_WIDTH = Units.inchesToMeters(315.5);

    private static final Map<Integer, Pose3d> APRIL_TAG_ORIGINALS = Map.of(1,
            new Pose3d(Units.feetToMeters(17),
                    Units.feetToMeters(8),
                    Units.inchesToMeters(34.5), new Rotation3d(0.0, 0.0, Math.PI)),
            2,
            new Pose3d(Units.feetToMeters(37),
                    Units.feetToMeters(8),
                    Units.inchesToMeters(34.5), new Rotation3d(0.0, 0.0, 0)),
            3,
            new Pose3d(Units.feetToMeters(37),
                    Units.feetToMeters(19),
                    Units.inchesToMeters(34.5), new Rotation3d(0.0, 0.0, 0)),
            4,
            new Pose3d(Units.feetToMeters(17),
                    Units.feetToMeters(19),
                    Units.inchesToMeters(34.5), new Rotation3d(0.0, 0.0, Math.PI))
    );

    public static final Pose3d reflectPosition(final Pose3d pose) {
        return new Pose3d(FIELD_LENGTH - pose.getTranslation().getX(), FIELD_WIDTH - pose.getTranslation().getY(),
                pose.getTranslation().getZ(),
                pose.getRotation().plus(new Rotation3d(0, 0, Math.PI)));
    }

    public static final Pose2d reflectPosition(final Pose2d pose) {
        return new Pose2d(FIELD_LENGTH - pose.getTranslation().getX(), FIELD_WIDTH - pose.getTranslation().getY(),
                pose.getRotation().plus(Rotation2d.fromRadians(Math.PI)));
    }
    public static final Map<Integer, Pose3d> getAprilTagsMap() {
        //return DriverStation.getAlliance() == DriverStation.Alliance.Blue ? APRIL_TAG_ORIGINALS : reflectAprilTags();
        return APRIL_TAG_ORIGINALS;
    }

    public static List<AprilTag> getAprilTagsList() {
        return getAprilTagsMap().entrySet().stream().map(entry -> new AprilTag(entry.getKey(), entry.getValue()))
                .toList();
    }
    public static AprilTagFieldLayout getCurrentFieldLayout() {
        return new AprilTagFieldLayout(getAprilTagsList(), FIELD_LENGTH, FIELD_WIDTH);
    }
   
    public static final Map<Integer, Pose3d> reflectAprilTags() {
        final Map<Integer, Pose3d> newMap = new HashMap<>();
        for (final Map.Entry<Integer, Pose3d> aprilTag : APRIL_TAG_ORIGINALS.entrySet()) {
            newMap.put(aprilTag.getKey(), reflectPosition(aprilTag.getValue()));
        }
        return newMap;
    }

    public static final Pose2d ORANGE_DUMP_ZONE = new Pose2d(5, 7, Rotation2d.fromDegrees(90));
    public static final Pose2d BLUE_DUMP_ZONE = new Pose2d(4.69, 0, Rotation2d.fromDegrees(-90));
}