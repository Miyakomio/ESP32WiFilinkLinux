/* Copyright (C) 2026 TinyDrone contributors
 * Licensed under the GNU General Public License, version 2 or later. */
package se.bitcraze.crazyfliecontrol2;

import android.content.ContentResolver;
import android.content.ContentValues;
import android.content.Context;
import android.graphics.Bitmap;
import android.graphics.BitmapFactory;
import android.media.MediaScannerConnection;
import android.media.Image;
import android.media.MediaCodec;
import android.media.MediaCodecInfo;
import android.media.MediaCodecList;
import android.media.MediaFormat;
import android.media.MediaMuxer;
import android.os.Build;
import android.os.Environment;
import android.net.Uri;
import android.provider.MediaStore;

import java.io.BufferedInputStream;
import java.io.DataInputStream;
import java.io.EOFException;
import java.io.File;
import java.io.FileInputStream;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.text.SimpleDateFormat;
import java.util.Arrays;
import java.util.Date;
import java.util.Locale;

/**
 * Reassembles the JPEG frames from a raw TinyDrone UDP recording and writes a
 * standard H.264 MP4 using Android's platform encoder.
 */
public final class VideoStreamDecoder {
    public interface ProgressListener {
        void onProgress(int completeFrames);
    }

    public static final class Result {
        public final Uri contentUri;
        public final String savedLocation;
        public final int frameCount;

        Result(Uri contentUri, String savedLocation, int frameCount) {
            this.contentUri = contentUri;
            this.savedLocation = savedLocation;
            this.frameCount = frameCount;
        }
    }

    private static final int PACKET_SIZE = 1400;
    private static final int HEADER_SIZE = 14;
    private static final int PAYLOAD_SIZE = PACKET_SIZE - HEADER_SIZE;
    private static final int MAX_FRAME_SIZE = 1024 * 1024;
    private static final int MAX_PACKETS =
            (MAX_FRAME_SIZE + PAYLOAD_SIZE - 1) / PAYLOAD_SIZE;
    private static final int MAX_PACKET_RECORD_SIZE = 65536;
    private static final int FRAME_RATE = 15;
    private static final int MAX_VIDEO_WIDTH = 640;
    private static final int MAX_VIDEO_HEIGHT = 480;
    private static final long MIN_FRAME_INTERVAL_MS = 1000L / FRAME_RATE;
    private static final long FRAME_INTERVAL_US = 1000000L / FRAME_RATE;

    private VideoStreamDecoder() {}

    public static Result decode(
            Context context,
            InputStream input,
            ProgressListener listener
    ) throws IOException {
        if (Build.VERSION.SDK_INT < Build.VERSION_CODES.LOLLIPOP) {
            throw new IOException("Video export requires Android 5.0 or newer");
        }

        File directory = VideoByteStreamRecorder.outputDirectory(context);
        if (!directory.exists() && !directory.mkdirs()) {
            throw new IOException("Cannot create video directory: " + directory);
        }
        String stamp = new SimpleDateFormat(
                "yyyyMMdd_HHmmss", Locale.US).format(new Date());
        String displayName = "TinyDrone_" + stamp + ".mp4";
        File partialFile = new File(directory, displayName + ".part");
        if (partialFile.exists() && !partialFile.delete()) {
            throw new IOException("Cannot replace existing temporary video");
        }

        H264Writer writer = null;
        try {
            DataInputStream recording = new DataInputStream(
                    new BufferedInputStream(input, 64 * 1024));
            readAndValidateHeader(recording);
            writer = new H264Writer(partialFile);
            FrameAssembler assembler = new FrameAssembler(writer, listener);

            while (true) {
                int type;
                try {
                    type = readIntLittleEndian(recording);
                } catch (EOFException end) {
                    break;
                }
                int length = readIntLittleEndian(recording);
                long timestampMs = readLongLittleEndian(recording);
                if (type != VideoByteStreamRecorder.RECORD_TYPE_VIDEO_PACKET ||
                        length <= 0 || length > MAX_PACKET_RECORD_SIZE) {
                    throw new IOException("Unsupported or corrupt recording record");
                }
                byte[] packet = new byte[length];
                recording.readFully(packet);
                assembler.accept(packet, timestampMs);
            }

            if (assembler.frameCount == 0) {
                throw new IOException("No complete JPEG frames found in recording");
            }
            writer.finish();
            writer.release();
            writer = null;

            Result result = publishToGallery(
                    context, partialFile, displayName, assembler.frameCount);
            if (partialFile.exists()) partialFile.delete();
            return result;
        } catch (IOException e) {
            if (writer != null) writer.release();
            if (partialFile.exists()) partialFile.delete();
            throw e;
        } catch (RuntimeException e) {
            if (writer != null) writer.release();
            if (partialFile.exists()) partialFile.delete();
            throw new IOException("Video encoding failed: " + e.getMessage(), e);
        }
    }

    private static Result publishToGallery(
            Context context,
            File source,
            String displayName,
            int frameCount
    ) throws IOException {
        if (Build.VERSION.SDK_INT >= 29) {
            ContentResolver resolver = context.getContentResolver();
            ContentValues values = new ContentValues();
            values.put(MediaStore.Video.Media.DISPLAY_NAME, displayName);
            values.put(MediaStore.Video.Media.MIME_TYPE, "video/mp4");
            // These string keys are available on Android 10+; compileSdk 28
            // does not expose their constants.
            values.put("relative_path", "Movies/TinyDrone");
            values.put("is_pending", 1);

            Uri uri = resolver.insert(
                    MediaStore.Video.Media.EXTERNAL_CONTENT_URI, values);
            if (uri == null) {
                throw new IOException("Cannot create gallery video entry");
            }
            OutputStream output = null;
            try {
                output = resolver.openOutputStream(uri);
                if (output == null) {
                    throw new IOException("Cannot open gallery video entry");
                }
                copyFile(source, output);
                output.flush();
            } catch (IOException e) {
                resolver.delete(uri, null, null);
                throw e;
            } finally {
                if (output != null) {
                    try {
                        output.close();
                    } catch (IOException ignored) {
                    }
                }
            }
            ContentValues published = new ContentValues();
            published.put("is_pending", 0);
            resolver.update(uri, published, null, null);
            return new Result(
                    uri,
                    "相册/Movies/TinyDrone/" + displayName,
                    frameCount);
        }

        File movies = Environment.getExternalStoragePublicDirectory(
                Environment.DIRECTORY_MOVIES);
        File directory = new File(movies, "TinyDrone");
        if (!directory.exists() && !directory.mkdirs()) {
            directory = VideoByteStreamRecorder.outputDirectory(context);
        }
        File destination = new File(directory, displayName);
        if (destination.exists() && !destination.delete()) {
            throw new IOException("Cannot replace existing gallery video");
        }
        if (!source.renameTo(destination)) {
            FileOutputStream output = new FileOutputStream(destination);
            try {
                copyFile(source, output);
            } finally {
                output.close();
            }
        }
        MediaScannerConnection.scanFile(
                context,
                new String[]{destination.getAbsolutePath()},
                new String[]{"video/mp4"},
                null);
        return new Result(
                Uri.fromFile(destination),
                destination.getAbsolutePath(),
                frameCount);
    }

    private static void copyFile(File source, OutputStream output)
            throws IOException {
        FileInputStream input = new FileInputStream(source);
        try {
            byte[] buffer = new byte[64 * 1024];
            int count;
            while ((count = input.read(buffer)) >= 0) {
                if (count > 0) output.write(buffer, 0, count);
            }
        } finally {
            input.close();
        }
    }

    private static void readAndValidateHeader(DataInputStream input)
            throws IOException {
        byte[] magic = new byte[VideoByteStreamRecorder.MAGIC.length];
        input.readFully(magic);
        if (!Arrays.equals(magic, VideoByteStreamRecorder.MAGIC)) {
            throw new IOException("Not a TinyDrone raw video recording");
        }
        int version = readIntLittleEndian(input);
        readIntLittleEndian(input);
        if (version != VideoByteStreamRecorder.FORMAT_VERSION) {
            throw new IOException("Unsupported recording version: " + version);
        }
    }

    private static final class FrameAssembler {
        private final H264Writer mWriter;
        private final ProgressListener mListener;
        private final boolean[] mReceived = new boolean[MAX_PACKETS];
        private byte[] mFrame = new byte[MAX_FRAME_SIZE];
        private long mFrameId = -1;
        private int mFrameSize;
        private int mExpectedPackets;
        private int mReceivedPackets;
        private long mFrameTimestampMs;
        private long mLastEncodedTimestampMs = -1;
        int frameCount;

        FrameAssembler(H264Writer writer, ProgressListener listener) {
            mWriter = writer;
            mListener = listener;
        }

        void accept(byte[] packet, long timestampMs) throws IOException {
            if (packet.length < HEADER_SIZE) return;
            ByteBuffer header = ByteBuffer.wrap(packet, 0, HEADER_SIZE)
                    .order(ByteOrder.LITTLE_ENDIAN);
            long frameId = header.getInt() & 0xffffffffL;
            long totalSizeLong = header.getInt() & 0xffffffffL;
            int totalPackets = header.getShort() & 0xffff;
            int sequence = header.getShort() & 0xffff;
            int dataLength = header.getShort() & 0xffff;
            int actualLength = packet.length - HEADER_SIZE;

            if (totalSizeLong < 4 || totalSizeLong > MAX_FRAME_SIZE ||
                    totalPackets == 0 || totalPackets > MAX_PACKETS ||
                    totalPackets != (totalSizeLong + PAYLOAD_SIZE - 1) / PAYLOAD_SIZE ||
                    sequence >= totalPackets || dataLength == 0 ||
                    dataLength > PAYLOAD_SIZE || dataLength != actualLength) {
                return;
            }

            if (frameId != mFrameId) {
                mFrameId = frameId;
                mFrameSize = (int) totalSizeLong;
                mExpectedPackets = totalPackets;
                mReceivedPackets = 0;
                mFrameTimestampMs = timestampMs;
                Arrays.fill(mReceived, false);
                if (mFrame.length < mFrameSize) {
                    mFrame = new byte[MAX_FRAME_SIZE];
                }
            }
            if (mFrameSize != (int) totalSizeLong ||
                    mExpectedPackets != totalPackets || mReceived[sequence]) {
                return;
            }

            int offset = sequence * PAYLOAD_SIZE;
            int expectedLength = Math.min(PAYLOAD_SIZE, mFrameSize - offset);
            if (offset < 0 || offset >= mFrameSize ||
                    dataLength != expectedLength) {
                return;
            }
            System.arraycopy(packet, HEADER_SIZE, mFrame, offset, dataLength);
            mReceived[sequence] = true;
            mReceivedPackets++;

            if (mReceivedPackets == mExpectedPackets) {
                if (isJpeg(mFrame, mFrameSize)) {
                    if (mLastEncodedTimestampMs < 0 ||
                            mFrameTimestampMs - mLastEncodedTimestampMs >=
                                    MIN_FRAME_INTERVAL_MS) {
                        mWriter.writeFrame(
                                mFrame, mFrameSize, mFrameTimestampMs);
                        mLastEncodedTimestampMs = mFrameTimestampMs;
                        frameCount++;
                    }
                    if (mListener != null &&
                            (frameCount == 1 || frameCount % 10 == 0)) {
                        mListener.onProgress(frameCount);
                    }
                }
                mFrameId = -1;
            }
        }
    }

    private static boolean isJpeg(byte[] data, int length) {
        return length >= 4 &&
                (data[0] & 0xff) == 0xff && (data[1] & 0xff) == 0xd8 &&
                (data[length - 2] & 0xff) == 0xff &&
                (data[length - 1] & 0xff) == 0xd9;
    }

    private static final class H264Writer {
        private static final long DEQUEUE_TIMEOUT_US = 10000;

        private final MediaCodec mCodec;
        private final MediaMuxer mMuxer;
        private final MediaCodec.BufferInfo mBufferInfo =
                new MediaCodec.BufferInfo();
        private final BitmapFactory.Options mBitmapOptions =
                new BitmapFactory.Options();
        private int mTrackIndex;
        private boolean mMuxerStarted;
        private boolean mSawEos;
        private long mFirstTimestampMs = -1;
        private long mLastPresentationUs = -1;
        private long mEncodedSampleCount;
        private int mWidth;
        private int mHeight;

        H264Writer(File outputFile) throws IOException {
            mCodec = createFlexibleEncoder();
            try {
                mMuxer = new MediaMuxer(
                        outputFile.getAbsolutePath(),
                        MediaMuxer.OutputFormat.MUXER_OUTPUT_MPEG_4);
            } catch (IOException e) {
                mCodec.release();
                throw e;
            }
            mBitmapOptions.inPreferredConfig = Bitmap.Config.ARGB_8888;
        }

        private static MediaCodec createFlexibleEncoder() throws IOException {
            MediaCodecList codecList = new MediaCodecList(
                    MediaCodecList.REGULAR_CODECS);
            for (MediaCodecInfo info : codecList.getCodecInfos()) {
                if (!info.isEncoder()) continue;
                try {
                    MediaCodecInfo.CodecCapabilities capabilities =
                            info.getCapabilitiesForType("video/avc");
                    for (int colorFormat : capabilities.colorFormats) {
                        if (colorFormat == MediaCodecInfo.CodecCapabilities
                                .COLOR_FormatYUV420Flexible) {
                            return MediaCodec.createByCodecName(info.getName());
                        }
                    }
                } catch (IllegalArgumentException ignored) {
                    // Codec does not implement an AVC encoder profile.
                }
            }
            throw new IOException(
                    "Device has no H.264 encoder with YUV420 input support");
        }

        void writeFrame(byte[] jpeg, int length, long timestampMs)
                throws IOException {
            Bitmap bitmap = BitmapFactory.decodeByteArray(
                    jpeg, 0, length, mBitmapOptions);
            if (bitmap == null) return;
            try {
                if (mWidth == 0) {
                    int sourceWidth = bitmap.getWidth();
                    int sourceHeight = bitmap.getHeight();
                    float scale = Math.min(
                            1.0f,
                            Math.min(
                                    (float) MAX_VIDEO_WIDTH / sourceWidth,
                                    (float) MAX_VIDEO_HEIGHT / sourceHeight));
                    mWidth = even(Math.round(sourceWidth * scale));
                    mHeight = even(Math.round(sourceHeight * scale));
                    if (mWidth < 16 || mHeight < 16) {
                        throw new IOException("Invalid camera image size");
                    }
                    configureEncoder();
                }
                if (mFirstTimestampMs < 0) mFirstTimestampMs = timestampMs;
                long presentationUs =
                        Math.max(0, timestampMs - mFirstTimestampMs) * 1000L;
                if (presentationUs <= mLastPresentationUs) {
                    presentationUs = mLastPresentationUs + FRAME_INTERVAL_US;
                }
                mLastPresentationUs = presentationUs;
                Bitmap encodeBitmap = bitmap;
                if (bitmap.getWidth() != mWidth ||
                        bitmap.getHeight() != mHeight) {
                    encodeBitmap = Bitmap.createScaledBitmap(
                            bitmap, mWidth, mHeight, true);
                }
                try {
                    queueBitmap(encodeBitmap, presentationUs);
                } finally {
                    if (encodeBitmap != bitmap) encodeBitmap.recycle();
                }
                drain(false);
            } finally {
                bitmap.recycle();
            }
        }

        void finish() throws IOException {
            if (mWidth == 0) throw new IOException("Recording contains no video frames");
            int inputIndex = mCodec.dequeueInputBuffer(100000);
            if (inputIndex < 0) {
                throw new IOException("Video encoder is not accepting the final frame");
            }
            mCodec.queueInputBuffer(
                    inputIndex, 0, 0,
                    mLastPresentationUs + FRAME_INTERVAL_US,
                    MediaCodec.BUFFER_FLAG_END_OF_STREAM);
            long deadlineMs = System.currentTimeMillis() + 5000;
            while (!mSawEos && System.currentTimeMillis() < deadlineMs) {
                drainOnce(true);
            }
            if (!mSawEos) {
                throw new IOException("Timed out finalizing video encoder");
            }
        }

        private void configureEncoder() throws IOException {
            MediaFormat format = MediaFormat.createVideoFormat(
                    "video/avc", mWidth, mHeight);
            format.setInteger(
                    MediaFormat.KEY_COLOR_FORMAT,
                    MediaCodecInfo.CodecCapabilities.COLOR_FormatYUV420Flexible);
            format.setInteger(
                    MediaFormat.KEY_BIT_RATE,
                    Math.max(1000000, mWidth * mHeight * 4));
            format.setInteger(
                    MediaFormat.KEY_MAX_INPUT_SIZE,
                    mWidth * mHeight * 3 / 2);
            format.setInteger(MediaFormat.KEY_FRAME_RATE, FRAME_RATE);
            format.setInteger(MediaFormat.KEY_I_FRAME_INTERVAL, 1);
            format.setInteger(
                    MediaFormat.KEY_PROFILE,
                    MediaCodecInfo.CodecProfileLevel.AVCProfileBaseline);
            format.setInteger(
                    MediaFormat.KEY_LEVEL,
                    MediaCodecInfo.CodecProfileLevel.AVCLevel31);
            mCodec.configure(
                    format, null, null, MediaCodec.CONFIGURE_FLAG_ENCODE);
            mCodec.start();
        }

        private void queueBitmap(Bitmap bitmap, long presentationUs)
                throws IOException {
            int inputIndex = mCodec.dequeueInputBuffer(100000);
            if (inputIndex < 0) {
                throw new IOException("Video encoder input timeout");
            }
            Image image = mCodec.getInputImage(inputIndex);
            if (image == null) {
                throw new IOException("Video encoder did not expose an input image");
            }
            try {
                fillYuvImage(image, bitmap);
            } finally {
                image.close();
            }
            final int yuvInputSize = mWidth * mHeight * 3 / 2;
            mCodec.queueInputBuffer(
                    inputIndex, 0, yuvInputSize, presentationUs, 0);
        }

        private void fillYuvImage(Image image, Bitmap bitmap) {
            int[] argb = new int[mWidth * mHeight];
            bitmap.getPixels(argb, 0, mWidth, 0, 0, mWidth, mHeight);
            Image.Plane[] planes = image.getPlanes();
            if (planes.length < 3) {
                throw new IllegalStateException("Unexpected YUV input image");
            }

            writeLuma(planes[0], argb);
            writeChroma(planes[1], argb, 1);
            writeChroma(planes[2], argb, 2);
        }

        private void writeLuma(Image.Plane plane, int[] argb) {
            ByteBuffer buffer = plane.getBuffer();
            int rowStride = plane.getRowStride();
            int pixelStride = plane.getPixelStride();
            int base = buffer.position();
            for (int y = 0; y < mHeight; y++) {
                int row = base + y * rowStride;
                for (int x = 0; x < mWidth; x++) {
                    int color = argb[y * mWidth + x];
                    int r = (color >> 16) & 0xff;
                    int g = (color >> 8) & 0xff;
                    int b = color & 0xff;
                    int luma = ((66 * r + 129 * g + 25 * b + 128) >> 8) + 16;
                    buffer.put(row + x * pixelStride, (byte) clamp(luma));
                }
            }
        }

        private void writeChroma(Image.Plane plane, int[] argb, int component) {
            ByteBuffer buffer = plane.getBuffer();
            int rowStride = plane.getRowStride();
            int pixelStride = plane.getPixelStride();
            int base = buffer.position();
            int chromaWidth = mWidth / 2;
            int chromaHeight = mHeight / 2;
            for (int y = 0; y < chromaHeight; y++) {
                int row = base + y * rowStride;
                for (int x = 0; x < chromaWidth; x++) {
                    int rgb00 = argb[(y * 2) * mWidth + x * 2];
                    int rgb01 = argb[(y * 2) * mWidth + x * 2 + 1];
                    int rgb10 = argb[(y * 2 + 1) * mWidth + x * 2];
                    int rgb11 = argb[(y * 2 + 1) * mWidth + x * 2 + 1];
                    int value;
                    if (component == 1) {
                        int r = average(rgb00, rgb01, rgb10, rgb11, 16);
                        int g = average(rgb00, rgb01, rgb10, rgb11, 8);
                        int b = average(rgb00, rgb01, rgb10, rgb11, 0);
                        value = ((-38 * r - 74 * g + 112 * b + 128) >> 8) + 128;
                    } else {
                        int r = average(rgb00, rgb01, rgb10, rgb11, 16);
                        int g = average(rgb00, rgb01, rgb10, rgb11, 8);
                        int b = average(rgb00, rgb01, rgb10, rgb11, 0);
                        value = ((112 * r - 94 * g - 18 * b + 128) >> 8) + 128;
                    }
                    buffer.put(row + x * pixelStride, (byte) clamp(value));
                }
            }
        }

        private void drain(boolean endOfStream) throws IOException {
            while (drainOnce(false)) {
                // Keep the encoder moving while frames are arriving.
            }
        }

        private boolean drainOnce(boolean endOfStream) throws IOException {
            int outputIndex = mCodec.dequeueOutputBuffer(
                    mBufferInfo, DEQUEUE_TIMEOUT_US);
            if (outputIndex == MediaCodec.INFO_TRY_AGAIN_LATER) {
                return false;
            }
            if (outputIndex == MediaCodec.INFO_OUTPUT_FORMAT_CHANGED) {
                if (mMuxerStarted) {
                    throw new IOException("Video format changed unexpectedly");
                }
                mTrackIndex = mMuxer.addTrack(mCodec.getOutputFormat());
                mMuxer.start();
                mMuxerStarted = true;
                return true;
            }
            if (outputIndex < 0) return true;

            ByteBuffer output = mCodec.getOutputBuffer(outputIndex);
            if (output == null) {
                mCodec.releaseOutputBuffer(outputIndex, false);
                return true;
            }
            if ((mBufferInfo.flags & MediaCodec.BUFFER_FLAG_CODEC_CONFIG) != 0) {
                mBufferInfo.size = 0;
            }
            if (mBufferInfo.size > 0) {
                if (!mMuxerStarted) {
                    throw new IOException("Video data arrived before output format");
                }
                // Some hardware encoders return zero PTS for Image input even
                // when a valid PTS was queued. Use a deterministic CFR timeline
                // so the MP4 always has a nonzero, player-compatible duration.
                mBufferInfo.presentationTimeUs =
                        mEncodedSampleCount * FRAME_INTERVAL_US;
                mEncodedSampleCount++;
                output.position(mBufferInfo.offset);
                output.limit(mBufferInfo.offset + mBufferInfo.size);
                mMuxer.writeSampleData(mTrackIndex, output, mBufferInfo);
            }
            boolean eos =
                    (mBufferInfo.flags & MediaCodec.BUFFER_FLAG_END_OF_STREAM) != 0;
            mCodec.releaseOutputBuffer(outputIndex, false);
            if (endOfStream && eos) mSawEos = true;
            return !eos;
        }

        void release() {
            try {
                mCodec.stop();
            } catch (RuntimeException ignored) {
            }
            try {
                mCodec.release();
            } catch (RuntimeException ignored) {
            }
            if (mMuxerStarted) {
                try {
                    mMuxer.stop();
                } catch (RuntimeException ignored) {
                }
            }
            try {
                mMuxer.release();
            } catch (RuntimeException ignored) {
            }
        }

        private static int average(
                int rgb00, int rgb01, int rgb10, int rgb11, int shift) {
            int value =
                    ((rgb00 >> shift) & 0xff) +
                    ((rgb01 >> shift) & 0xff) +
                    ((rgb10 >> shift) & 0xff) +
                    ((rgb11 >> shift) & 0xff);
            return value / 4;
        }

        private static int clamp(int value) {
            return value < 0 ? 0 : (value > 255 ? 255 : value);
        }

        private static int even(int value) {
            return Math.max(2, value & ~1);
        }
    }

    private static int readIntLittleEndian(DataInputStream input)
            throws IOException {
        int b0 = input.read();
        if (b0 < 0) throw new EOFException();
        int b1 = input.readUnsignedByte();
        int b2 = input.readUnsignedByte();
        int b3 = input.readUnsignedByte();
        return b0 | (b1 << 8) | (b2 << 16) | (b3 << 24);
    }

    private static long readLongLittleEndian(DataInputStream input)
            throws IOException {
        long low = readIntLittleEndian(input) & 0xffffffffL;
        long high = readIntLittleEndian(input) & 0xffffffffL;
        return low | (high << 32);
    }
}
