package org.texastorque.auto;

import org.texastorque.auto.sequences.IntakeBucket;
import org.texastorque.torquelib.auto.*;

public final class AutoManager extends TorqueAutoManager {
    private static volatile AutoManager instance;


    public AutoManager() {
        super(true);
    }

    @Override
    public final void init() {
        addSequence(new IntakeBucket());
    }

    /**
     * Get the AutoManager instance
     *
     * @return AutoManager
     */
    public static final synchronized AutoManager getInstance() {
        return instance == null ? instance = new AutoManager() : instance;
    }
}
