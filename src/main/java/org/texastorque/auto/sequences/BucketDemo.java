package org.texastorque.auto.sequences;

import org.texastorque.Subsystems;
import org.texastorque.torquelib.auto.TorqueSequence;
import org.texastorque.torquelib.auto.commands.TorqueRunSequence;
import org.texastorque.torquelib.auto.commands.TorqueRunWhile;

public class BucketDemo extends TorqueSequence implements Subsystems {
    public BucketDemo() {
        addBlock(new TorqueRunWhile(new TorqueRunSequence(new FindBucket()), () -> true));
        // addBlock(new TorqueRunSequence(new FindBucket()));
        // addBlock(new TorqueRunSequence(new FindBucket()));
    }
}
