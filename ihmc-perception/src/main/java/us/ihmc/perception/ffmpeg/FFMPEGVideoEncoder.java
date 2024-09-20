package us.ihmc.perception.ffmpeg;

import org.bytedeco.ffmpeg.avformat.AVFormatContext;
import org.bytedeco.ffmpeg.avformat.AVOutputFormat;
import org.bytedeco.ffmpeg.avformat.AVStream;
import org.bytedeco.javacpp.Pointer;
import us.ihmc.commons.Conversions;

import static org.bytedeco.ffmpeg.global.avcodec.avcodec_parameters_from_context;
import static org.bytedeco.ffmpeg.global.avutil.av_make_q;

public abstract class FFMPEGVideoEncoder extends FFMPEGEncoder
{
   private int colorConversion = -1;
   private long firstFrameTime = -1L;

   public FFMPEGVideoEncoder(AVOutputFormat outputFormat,
                             String preferredEncoderName,
                             int bitRate,
                             int outputWidth,
                             int outputHeight,
                             int groupOfPicturesSize,
                             int maxBFrames)
   {
      super(outputFormat, preferredEncoderName, bitRate);

      // Use nanosecond precision for timebase
      timeBase = av_make_q(1, (int) Conversions.secondsToNanoseconds(1.0));

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

      error = avcodec_parameters_from_context(stream.codecpar(), encoderContext);
      FFMPEGTools.checkNegativeError(error, "Copying parameters from codec to stream");

      return stream;
   }

   /**
    * Assign the frame to be encoded with the provided image.
    * @param image Pointer to an object containing the image to be encoded
    */
   @Override
   public final void setNextFrame(Pointer image)
   {
      // Set the frame PTS based on time since first frame
      long currentTime = System.nanoTime();
      if (firstFrameTime < 0L)
         firstFrameTime = currentTime;
      long timeElapsed = currentTime - firstFrameTime;
      frameToEncode.pts(timeElapsed);

      // Set the AVFrame's image
      prepareFrameForEncoding(image);
   }

   protected int getColorConversion()
   {
      return colorConversion;
   }

   /**
    * Prepare the {@link #frameToEncode} to be encoded.
    * This includes any preprocessing (e.g. color conversions, resizing, etc.)
    * that leads up to the frame being encoded.
    * @param imageToEncode Pointer to an object containing the image to be encoded
    */
   protected abstract void prepareFrameForEncoding(Pointer imageToEncode);
}
