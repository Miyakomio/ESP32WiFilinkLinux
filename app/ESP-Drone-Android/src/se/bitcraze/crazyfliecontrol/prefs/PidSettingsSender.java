package se.bitcraze.crazyfliecontrol.prefs;

import android.content.SharedPreferences;

import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.InetAddress;

import se.bitcraze.crazyflie.lib.crtp.PidSettingsPacket;
import se.bitcraze.crazyfliecontrol2.EspUdpDriver;

/** Sends one CRC-protected PID configuration packet to the drone. */
public final class PidSettingsSender {
    private PidSettingsSender() {
    }

    public static void send(SharedPreferences preferences) throws Exception {
        float[] gains = new float[PidSettingsPacket.GAIN_COUNT];
        for (int i = 0; i < gains.length; i++) {
            String value = preferences.getString(
                    PreferencesActivity.KEY_PREF_PID_GAINS[i],
                    PreferencesActivity.DEFAULT_PID_GAINS[i]);
            gains[i] = Float.parseFloat(value);
            if (Float.isNaN(gains[i]) || Float.isInfinite(gains[i]) ||
                    gains[i] < 0.0f || gains[i] > 100.0f) {
                throw new IllegalArgumentException(
                        "PID gain out of range: " + gains[i]);
            }
            int loop = i / 3;
            float min = Float.parseFloat(preferences.getString(
                    PreferencesActivity.KEY_PREF_PID_RANGES[loop] + "_min",
                    PreferencesActivity.DEFAULT_PID_RANGE_MIN[loop]));
            float max = Float.parseFloat(preferences.getString(
                    PreferencesActivity.KEY_PREF_PID_RANGES[loop] + "_max",
                    PreferencesActivity.DEFAULT_PID_RANGE_MAX[loop]));
            if (Float.isNaN(min) || Float.isInfinite(min) ||
                    Float.isNaN(max) || Float.isInfinite(max) ||
                    min < 0.0f || max > 100.0f || min >= max) {
                throw new IllegalArgumentException(
                        "Invalid PID range for loop " + loop);
            }
            if (gains[i] < min || gains[i] > max) {
                throw new IllegalArgumentException(
                        "PID gain outside configured range: " + gains[i]);
            }
        }

        byte[] payload = new PidSettingsPacket(gains).toByteArray();
        byte[] packet = new byte[payload.length + 1];
        System.arraycopy(payload, 0, packet, 0, payload.length);
        int checksum = 0;
        for (byte value : payload) {
            checksum += value & 0xff;
        }
        packet[packet.length - 1] = (byte) checksum;

        DatagramSocket socket = new DatagramSocket();
        try {
            InetAddress address =
                    InetAddress.getByName(EspUdpDriver.DEVICE_ADDRESS);
            socket.send(new DatagramPacket(
                    packet,
                    packet.length,
                    address,
                    EspUdpDriver.DEVICE_PORT));
        } finally {
            socket.close();
        }
    }
}
