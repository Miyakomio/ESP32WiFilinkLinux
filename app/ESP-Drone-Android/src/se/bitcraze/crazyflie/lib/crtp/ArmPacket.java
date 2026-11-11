package se.bitcraze.crazyflie.lib.crtp;

import java.nio.ByteBuffer;

/**
 * Requests motor arming without changing the commander set-point packet.
 * Channel 2 is reserved for this state flag.
 */
public class ArmPacket extends CrtpPacket {
    private final boolean mArmed;

    public ArmPacket(boolean armed) {
        super(2, CrtpPort.COMMANDER);
        mArmed = armed;
    }

    @Override
    protected void serializeData(ByteBuffer buffer) {
        buffer.put((byte) (mArmed ? 1 : 0));
    }

    @Override
    protected int getDataByteCount() {
        return 1;
    }

    @Override
    public String toString() {
        return "ArmPacket: armed: " + mArmed;
    }
}
