package se.bitcraze.crazyflie.lib.crtp;

import java.nio.ByteBuffer;

/**
 * Sends the five TinyDrone PID loops without modifying the existing
 * commander/hover/arm packet channels.
 */
public class PidSettingsPacket extends CrtpPacket {
    public static final int LOOP_ROLL_ANGLE = 0;
    public static final int LOOP_PITCH_ANGLE = 1;
    public static final int LOOP_YAW_RATE = 2;
    public static final int LOOP_ROLL_RATE = 3;
    public static final int LOOP_PITCH_RATE = 4;
    public static final int LOOP_COUNT = 5;
    public static final int GAIN_COUNT = LOOP_COUNT * 3;

    private final float[] mGains;

    public PidSettingsPacket(float[] gains) {
        super(3, CrtpPort.COMMANDER);
        if (gains == null || gains.length != GAIN_COUNT) {
            throw new IllegalArgumentException(
                    "Expected " + GAIN_COUNT + " PID gains");
        }
        mGains = gains.clone();
    }

    @Override
    protected void serializeData(ByteBuffer buffer) {
        buffer.put((byte) 0x50);
        buffer.put((byte) 1);
        buffer.put((byte) LOOP_COUNT);
        buffer.put((byte) 0);
        for (float gain : mGains) {
            buffer.putFloat(gain);
        }
    }

    @Override
    protected int getDataByteCount() {
        return 4 + GAIN_COUNT * 4;
    }
}
