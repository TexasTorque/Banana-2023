package org.texastorque;

import org.texastorque.auto.AutoManager;
import org.texastorque.toast.lib.pipelines.ObjectDetector;
import org.texastorque.toast.lib.pipelines.ObjectDetector.DetectedObject;
import org.texastorque.torquelib.base.*;

public final class Robot extends TorqueRobotBase implements Subsystems {

    // private DetectedObject someBucket = ObjectDetector.emptyObject;

    public Robot() {
        super(Input.getInstance(), AutoManager.getInstance());

        addSubsystem(drivebase);
        addSubsystem(elevator);
        addSubsystem(wrist);
        addSubsystem(intake);
        addSubsystem(lights);

        Debug.initDashboard();
    }


    // public void burnBucketToRetina(final DetectedObject someBucket) {
    // this.someBucket = someBucket;
    // }

    // public DetectedObject getBucket


}
