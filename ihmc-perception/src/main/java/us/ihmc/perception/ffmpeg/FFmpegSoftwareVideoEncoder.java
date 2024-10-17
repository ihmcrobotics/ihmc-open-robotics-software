package us.ihmc.perception.ffmpeg;

import org.bytedeco.ffmpeg.avformat.AVOutputFormat;
import org.bytedeco.ffmpeg.avutil.AVDictionary;
import org.bytedeco.ffmpeg.avutil.AVFrame;
import org.bytedeco.ffmpeg.swscale.SwsContext;
import org.bytedeco.javacpp.DoublePointer;
import org.bytedeco.javacpp.Pointer;
import org.bytedeco.opencv.global.opencv_imgproc;
import org.bytedeco.opencv.opencv_core.Mat;
import org.bytedeco.opencv.opencv_core.MatVector;
import org.bytedeco.opencv.opencv_core.Size;
import us.ihmc.perception.RawImage;

import static org.bytedeco.ffmpeg.global.avutil.*;
import static org.bytedeco.ffmpeg.global.swscale.*;

/**
 * Software implementation of a video encoder that uses LibAV.
 */
public class FFmpegSoftwareVideoEncoder extends FFmpegVideoEncoder
{
   private static final int SCALE_METHOD = SWS_BICUBIC;
   private SwsContext swsContext;

   private final AVFrame frameToScale;
   private final Mat tempMat;

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
    * @param outputPixelFormat Pixel format of the output video (Must be one of avutil.AV_PIX_FMT_*).
    * @param groupOfPicturesSize Number of frames between key frames.
    * @param maxBFrames Maximum number of B frames in every GOP.
    * @param inputPixelFormat Pixel format of input images (one of avutil.AV_PIX_FMT_*).
    */
   public FFmpegSoftwareVideoEncoder(AVOutputFormat outputFormat,
                                     String preferredEncoderName,
                                     int bitRate,
                                     int outputWidth,
                                     int outputHeight,
                                     int outputPixelFormat,
                                     int groupOfPicturesSize,
                                     int maxBFrames,
                                     int inputPixelFormat)
   {
      super(outputFormat, preferredEncoderName, bitRate, outputWidth, outputHeight, groupOfPicturesSize, maxBFrames);

      tempMat = new Mat();

      frameToScale = av_frame_alloc();
      FFmpegTools.checkPointer(frameToScale, "Allocating input frame");
      frameToScale.format(inputPixelFormat);
      // this frame's width & height initialized upon reception of first image
      frameToScale.width(-1);
      frameToScale.height(-1);

      // Set the output pixel format and allocate memory for the frame
      frameToEncode.format(outputPixelFormat);
      error = av_frame_get_buffer(frameToEncode, 0);
      FFmpegTools.checkNegativeError(error, "Getting next frame buffer");

      encoderContext.pix_fmt(outputPixelFormat);
   }

   @Override
   public void setNextFrame(RawImage image)
   {
      setNextFrameAcquisitionTime(image.getAcquisitionTime().toEpochMilli());
      setNextFrame(image.getCpuImageMat());
   }

   @Override
   protected void prepareFrameForEncoding(Pointer imageToEncode)
   {
      if (imageToEncode instanceof Mat mat)
      {
         MatVector matVector = new MatVector(mat);
         prepareFrameForEncoding(matVector);
         matVector.close();
      }
      else if (imageToEncode instanceof  MatVector matVector)
         prepareFrameForEncoding(matVector);
   }

   protected void prepareFrameForEncoding(MatVector imagePlanes)
   {
      int requestedColorConversion = getColorConversion();
      if (requestedColorConversion >= 0) // if a color conversion is requested
      {
         // Convert color an assign to another gpu mat to avoid changing the input data
         for (int i = 0; i < imagePlanes.size(); ++i)
         {
            opencv_imgproc.cvtColor(imagePlanes.get(i), tempMat, requestedColorConversion);
            imagePlanes.put(i, tempMat);
         }
      }

      // If the input and output dimensions or pixel formats don't match
      if (frameToEncode.width() != imagePlanes.get(0).cols() || frameToEncode.height() != imagePlanes.get(0).rows()
          || frameToScale.format() != frameToEncode.format())
      {
         if (swsContext == null) // Initialize frame scaling for first time
            initializeScaling(imagePlanes.get(0).size());

         // Scale the frame and put data in the frame to encode
         for (int i = 0; i < imagePlanes.size(); ++i)
            frameToScale.data(i, imagePlanes.get(i).data());
         error = sws_scale_frame(swsContext, frameToEncode, frameToScale);
         FFmpegTools.checkNegativeError(error, "Scaling frame");
      }
      else // No scaling needed; put data directly into frame to encode
      {
         for (int i = 0; i < imagePlanes.size(); ++i)
            frameToEncode.data(i, imagePlanes.get(i).data());
      }
   }

   private void initializeScaling(Size inputImageSize)
   {
      // Initialize an intermediate frame
      frameToScale.width(inputImageSize.width());
      frameToScale.height(inputImageSize.height());
      error = av_frame_get_buffer(frameToScale, 0);
      FFmpegTools.checkNegativeError(error, "Getting input frame buffer");

      // Initialize the scaling context
      swsContext = sws_getContext(frameToScale.width(),
                                  frameToScale.height(),
                                  frameToScale.format(),
                                  frameToEncode.width(),
                                  frameToEncode.height(),
                                  frameToEncode.format(),
                                  SCALE_METHOD,
                                  null,
                                  null,
                                  (DoublePointer) null);
      FFmpegTools.checkPointer(swsContext, "Initializing swsContext");
   }

   @Override
   public void destroy()
   {
      super.destroy();

      av_frame_free(frameToScale);
      frameToScale.close();

      tempMat.close();

      if (swsContext != null)
      {
         sws_freeContext(swsContext);
         swsContext.close();
      }
   }
}
