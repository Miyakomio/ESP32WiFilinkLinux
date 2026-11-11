package se.bitcraze.crazyflie.lib.crtp;

import java.nio.ByteBuffer;

/**
 * Sends the requested hover state without changing the normal commander
 * packet. Channel 1 is reserved for this additional control flag.
 */
public class HoverPacket extends CrtpPacket {
    private final boolean mEnabled;

    public HoverPacket(boolean enabled) {
        super(1, CrtpPort.COMMANDER);
        mEnabled = enabled;
    }

    @Override
    protected void serializeData(ByteBuffer buffer) {
        buffer.put((byte) (mEnabled ? 1 : 0));
    }

    @Override
    protected int getDataByteCount() {
        return 1;
    }

    @Override
    public String toString() {
        return "HoverPacket: enabled: " + mEnabled;
    }
}
