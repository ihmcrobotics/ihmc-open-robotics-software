package us.ihmc.perception.ffmpeg;

import org.bytedeco.ffmpeg.avformat.AVFormatContext;
import org.bytedeco.ffmpeg.avformat.AVOutputFormat;
import org.bytedeco.ffmpeg.avformat.AVStream;
import org.bytedeco.ffmpeg.avutil.AVBufferRef;
import org.bytedeco.ffmpeg.avutil.AVDictionary;
import org.bytedeco.ffmpeg.avutil.AVFrameSideData;
import org.bytedeco.javacpp.BytePointer;
import org.bytedeco.javacpp.Pointer;
import us.ihmc.commons.Conversions;
import us.ihmc.perception.RawImage;

import java.util.UUID;

import static org.bytedeco.ffmpeg.global.avcodec.avcodec_parameters_from_context;
import static org.bytedeco.ffmpeg.global.avutil.*;

/**
 * A video encoder that uses LibAV.
 */
public abstract class FFmpegVideoEncoder extends FFmpegEncoder
{
   private final BytePointer uuid = new BytePointer(16);

   private int colorConversion = -1;
   private long firstFrameTime = -1L;
   private long nextFrameTime = -1L;

   /**
    * Creates a new video encoder. Must call {@link FFmpegEncoder#initialize(AVDictionary)} after this.
    * @param outputFormat The format of encoded output
    *                     (see: <a href="https://ffmpeg.org/ffmpeg-formats.html">FFmpeg Formats Documentation</a>,
    *                     or run {@code ffmpeg -hide_banner -formats} to see a list of formats).
    * @param preferredEncoderName Name of the encoder to use
    *                             (run {@code ffmpeg -hide_banner -encoders} to see a list of encoders, though not all encoders may be available).
    * @param bitRate Target output bit rate.
    * @param outputWidth Width of the output video. If the output dimensions don't match that of the input, the input will be rescaled before encoding.
    * @param outputHeight Height of the output video. If the output dimensions don't match that of the input, the input will be rescaled before encoding.
    * @param groupOfPicturesSize Number of frames between key frames.
    * @param maxBFrames Maximum number of B frames in every GOP.
    */
   public FFmpegVideoEncoder(AVOutputFormat outputFormat,
                             String preferredEncoderName,
                             int bitRate,
                             int outputWidth,
                             int outputHeight,
                             int groupOfPicturesSize,
                             int maxBFrames)
   {
      super(outputFormat, preferredEncoderName, bitRate);

      UUID randomUUID = UUID.randomUUID();
      uuid.putLong(randomUUID.getMostSignificantBits());
      uuid.putLong(randomUUID.getLeastSignificantBits());
      uuid.limit(16);

      // Use nanosecond precision for timebase
      timeBase = av_make_q(1, (int) Conversions.secondsToMilliseconds(1.0));

      // Set encoder context parameters
      encoderContext.time_base(timeBase);
      encoderContext.bit_rate(bitRate);
      encoderContext.codec_id(encoder.id());
      encoderContext.width(outputWidth);
      encoderContext.height(outputHeight);
      encoderContext.gop_size(groupOfPicturesSize);
      encoderContext.max_b_frames(maxBFrames);

      // Set size of frame to encode
      frameToEncode.width(outputWidth);
      frameToEncode.height(outputHeight);
   }

   /**
    * OpenCV color conversion (i.e. one of {@code opencv_imgproc.COLOR_*}) to apply such that the input image matches the input pixel format.
    * This may be useful as some encoders only accept certain pixel formats.
    * @param opencvImgprocColorConversion One of {@code opencv_imgproc.COLOR_*}
    */
   public void setIntermediateColorConversion(int opencvImgprocColorConversion)
   {
      colorConversion = opencvImgprocColorConversion;
   }

   /**
    * Get a new stream to go with the provided output context.
    * @param outputContext Output context of the stream to be created.
    * @return An AVStream from the encoder with the provided output context.
    */
   @Override
   public AVStream newStream(AVFormatContext outputContext)
   {
      AVStream stream = super.newStream(outputContext);
      stream.start_time(0L); // This encoder always starts PTS with 0

      error = avcodec_parameters_from_context(stream.codecpar(), encoderContext);
      FFmpegTools.checkNegativeError(error, "Copying parameters from codec to stream");

      return stream;
   }

   /**
    * Assign the frame to be encoded with the provided image.
    * @param image Pointer to an object containing the image to be encoded
    */
   @Override
   public final void setNextFrame(Pointer image)
   {  // If next frame time has not been set, use current time
      if (nextFrameTime < 0L)
         nextFrameTime = System.currentTimeMillis();

      // Set the frame PTS based on time since first frame
      if (firstFrameTime < 0L)
         firstFrameTime = nextFrameTime;
      long timeElapsed = nextFrameTime - firstFrameTime;
      frameToEncode.pts(timeElapsed);

      // Set the AVFrame's image
      prepareFrameForEncoding(image);

      // Reset next frame time to be invalid
      nextFrameTime = -1L;
   }

   /**
    * Set the acquisition time of the next frame to be provided.
    * This must be called BEFORE {@link #setNextFrame(Pointer)}.
    * @param nextFrameAcquisitionTime Milliseconds since the Unix epoch when the frame was acquired.
    */
   public void setNextFrameAcquisitionTime(long nextFrameAcquisitionTime)
   {
      nextFrameTime = nextFrameAcquisitionTime;
   }

   public void setNextFrameSideData(BytePointer data)
   {
      AVBufferRef seiBuffer = av_buffer_alloc(16 + data.limit());
      Pointer.memcpy(seiBuffer.data().position(0), uuid.position(0), 16);
      Pointer.memcpy(seiBuffer.data().position(16), data.position(0), data.limit());

      av_frame_remove_side_data(frameToEncode, AV_FRAME_DATA_SEI_UNREGISTERED);
      AVFrameSideData sideData = av_frame_new_side_data_from_buf(frameToEncode, AV_FRAME_DATA_SEI_UNREGISTERED, seiBuffer);
      FFmpegTools.checkPointer(sideData, "Adding side data to AVFrame");
   }

   public long getStartTime()
   {
      return firstFrameTime;
   }

   @Override
   public void destroy()
   {
      super.destroy();

      uuid.close();
   }

   protected int getColorConversion()
   {
      return colorConversion;
   }

   /**
    * Assign the frame to be encoded with the provided RawImage.
    * The next frame acquisition time is set using the RawImage's acquisition time.
    * @param image {@link RawImage} containing the data to be encoded.
    */
   public abstract void setNextFrame(RawImage image);

   /**
    * Prepare the {@link #frameToEncode} to be encoded.
    * This includes any preprocessing (e.g. color conversions, resizing, etc.)
    * that leads up to the frame being encoded.
    * @param imageToEncode Pointer to an object containing the image to be encoded
    */
   protected abstract void prepareFrameForEncoding(Pointer imageToEncode);
}
