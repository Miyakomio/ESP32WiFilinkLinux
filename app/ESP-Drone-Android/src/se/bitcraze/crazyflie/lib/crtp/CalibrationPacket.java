package se.bitcraze.crazyflie.lib.crtp;

import java.nio.ByteBuffer;

/** Requests a gyro-bias and level calibration. */
public class CalibrationPacket extends CrtpPacket {
    private static final int MAGIC = 0xCA;
    private static final int VERSION = 1;

    public CalibrationPacket() {
        super(0, CrtpPort.HIGH_LEVEL_COMMANDER);
    }

    @Override
    protected void serializeData(ByteBuffer buffer) {
        buffer.put((byte) MAGIC);
        buffer.put((byte) VERSION);
    }

    @Override
    protected int getDataByteCount() {
        return 2;
    }

    @Override
    public String toString() {
        return "CalibrationPacket";
    }
}
