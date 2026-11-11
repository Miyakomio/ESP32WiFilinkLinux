/* Copyright (C) 2026 TinyDrone contributors
 * Licensed under the GNU General Public License, version 2 or later. */
package se.bitcraze.crazyfliecontrol2;

import android.content.Context;
import android.os.Environment;
import android.util.Log;

import java.io.BufferedOutputStream;
import java.io.DataOutputStream;
import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;
import java.nio.charset.Charset;
import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.Locale;
import java.util.concurrent.ArrayBlockingQueue;
import java.util.concurrent.BlockingQueue;

/**
 * Records the raw UDP video packets without decoding or discarding fragments.
 *
 * <p>File layout: an 8-byte magic ("TDREC001"), a 32-bit version and a 32-bit
 * reserved field, followed by records of four little-endian integers: packet
 * type, payload length, timestamp high word and timestamp low word.</p>
 */
public final class VideoByteStreamRecorder {
    private static final String TAG = "VideoByteStreamRecorder";
    public static final String FILE_EXTENSION = ".tdrec";
    public static final int FORMAT_VERSION = 1;
    public static final int RECORD_TYPE_VIDEO_PACKET = 1;
    public static final byte[] MAGIC =
            "TDREC001".getBytes(Charset.forName("US-ASCII"));

    private static final int QUEUE_CAPACITY = 512;

    private final BlockingQueue<PacketRecord> mQueue =
            new ArrayBlockingQueue<>(QUEUE_CAPACITY);
    private final Object mLifecycleLock = new Object();

    private volatile boolean mRecording;
    private volatile File mFile;
    private Thread mWriterThread;
    private DataOutputStream mOutput;
    private long mPacketCount;
    private long mByteCount;
    private long mDroppedPackets;
    private long mStartedAtMs;

    public static final class Result {
        public final File file;
        public final long packets;
        public final long bytes;
        public final long droppedPackets;
        public final long durationMs;

        Result(File file, long packets, long bytes, long droppedPackets,
               long durationMs) {
            this.file = file;
            this.packets = packets;
            this.bytes = bytes;
            this.droppedPackets = droppedPackets;
            this.durationMs = durationMs;
        }
    }

    private static final class PacketRecord {
        final byte[] data;
        final long timestampMs;

        PacketRecord(byte[] data, long timestampMs) {
            this.data = data;
            this.timestampMs = timestampMs;
        }
    }

    public boolean isRecording() {
        return mRecording;
    }

    public File currentFile() {
        return mFile;
    }

    public File start(Context context) throws IOException {
        synchronized (mLifecycleLock) {
            if (mRecording) return mFile;

            File directory = outputDirectory(context);
            if (!directory.exists() && !directory.mkdirs()) {
                throw new IOException("Cannot create recording directory: " + directory);
            }

            String stamp = new SimpleDateFormat(
                    "yyyyMMdd_HHmmss", Locale.US).format(new Date());
            File file = new File(directory, "video_" + stamp + FILE_EXTENSION);
            mOutput = new DataOutputStream(new BufferedOutputStream(
                    new FileOutputStream(file), 128 * 1024));
            mOutput.write(MAGIC);
            writeIntLittleEndian(mOutput, FORMAT_VERSION);
            writeIntLittleEndian(mOutput, 0);
            mOutput.flush();

            mFile = file;
            mPacketCount = 0;
            mByteCount = 0;
            mDroppedPackets = 0;
            mStartedAtMs = System.currentTimeMillis();
            mQueue.clear();
            mRecording = true;
            mWriterThread = new Thread(new Runnable() {
                @Override public void run() {
                    writerLoop();
                }
            }, "TinyDrone-video-record");
            mWriterThread.start();
            return file;
        }
    }

    public void enqueuePacket(byte[] data, int offset, int length, long timestampMs) {
        if (!mRecording || data == null || length <= 0) return;
        byte[] copy = new byte[length];
        System.arraycopy(data, offset, copy, 0, length);
        if (!mQueue.offer(new PacketRecord(copy, timestampMs))) {
            mDroppedPackets++;
        }
    }

    public Result stop() {
        synchronized (mLifecycleLock) {
            if (!mRecording && mFile == null) return null;
            mRecording = false;
            Thread writer = mWriterThread;
            if (writer != null) {
                try {
                    writer.join(5000);
                } catch (InterruptedException e) {
                    Thread.currentThread().interrupt();
                }
            }
            closeOutput();
            mWriterThread = null;
            Result result = new Result(
                    mFile,
                    mPacketCount,
                    mByteCount,
                    mDroppedPackets,
                    Math.max(0, System.currentTimeMillis() - mStartedAtMs));
            mFile = null;
            return result;
        }
    }

    private void writerLoop() {
        try {
            while (mRecording || !mQueue.isEmpty()) {
                PacketRecord record = mQueue.poll();
                if (record == null) {
                    try {
                        Thread.sleep(10);
                    } catch (InterruptedException e) {
                        if (!mRecording) break;
                    }
                    continue;
                }

                writeIntLittleEndian(mOutput, RECORD_TYPE_VIDEO_PACKET);
                writeIntLittleEndian(mOutput, record.data.length);
                writeLongLittleEndian(mOutput, record.timestampMs);
                mOutput.write(record.data);
                mPacketCount++;
                mByteCount += record.data.length;
            }
        } catch (IOException e) {
            Log.e(TAG, "Raw video recording failed", e);
            mRecording = false;
        } finally {
            closeOutput();
        }
    }

    private void closeOutput() {
        DataOutputStream output = mOutput;
        mOutput = null;
        if (output != null) {
            try {
                output.flush();
                output.close();
            } catch (IOException e) {
                Log.w(TAG, "Closing recording failed", e);
            }
        }
    }

    public static File outputDirectory(Context context) {
        File external = context.getExternalFilesDir(Environment.DIRECTORY_MOVIES);
        File base = external != null
                ? external
                : new File(context.getFilesDir(), "Movies");
        return new File(base, "TinyDrone");
    }

    private static void writeIntLittleEndian(DataOutputStream output, int value)
            throws IOException {
        output.writeByte(value & 0xff);
        output.writeByte((value >>> 8) & 0xff);
        output.writeByte((value >>> 16) & 0xff);
        output.writeByte((value >>> 24) & 0xff);
    }

    private static void writeLongLittleEndian(DataOutputStream output, long value)
            throws IOException {
        writeIntLittleEndian(output, (int) (value & 0xffffffffL));
        writeIntLittleEndian(output, (int) ((value >>> 32) & 0xffffffffL));
    }
}
