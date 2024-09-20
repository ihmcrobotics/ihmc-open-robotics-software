package us.ihmc.perception.ffmpeg;

import org.bytedeco.ffmpeg.avformat.AVOutputFormat;
import org.bytedeco.ffmpeg.avutil.AVFrame;
import org.bytedeco.ffmpeg.swscale.SwsContext;
import org.bytedeco.javacpp.DoublePointer;
import org.bytedeco.javacpp.Pointer;
import org.bytedeco.opencv.global.opencv_imgproc;
import org.bytedeco.opencv.opencv_core.Mat;
import org.bytedeco.opencv.opencv_core.Size;

import static org.bytedeco.ffmpeg.global.avutil.*;
import static org.bytedeco.ffmpeg.global.swscale.*;

public class FFMPEGSoftwareVideoEncoder extends FFMPEGVideoEncoder
{
   private static final int SCALE_METHOD = SWS_BICUBIC;
   private SwsContext swsContext;

   private final AVFrame frameToScale;
   private final Mat tempMat;

   public FFMPEGSoftwareVideoEncoder(AVOutputFormat outputFormat,
                                     String preferredCodecName,
                                     int bitRate,
                                     int outputWidth,
                                     int outputHeight,
                                     int outputPixelFormat,
                                     int groupOfPicturesSize,
                                     int maxBFrames,
                                     int inputPixelFormat)
   {
      super(outputFormat, preferredCodecName, bitRate, outputWidth, outputHeight, groupOfPicturesSize, maxBFrames);

      tempMat = new Mat();

      frameToScale = av_frame_alloc();
      FFMPEGTools.checkPointer(frameToScale, "Allocating input frame");
      frameToScale.format(inputPixelFormat);
      // this frame's width & height initialized upon reception of first image
      frameToScale.width(-1);
      frameToScale.height(-1);

      // Set the output pixel format and allocate memory for the frame
      frameToEncode.format(outputPixelFormat);
      error = av_frame_get_buffer(frameToEncode, 0);
      FFMPEGTools.checkNegativeError(error, "Getting next frame buffer");

      encoderContext.pix_fmt(outputPixelFormat);
   }

   @Override
   protected void prepareFrameForEncoding(Pointer imageMatToEncode)
   {
      Mat image = new Mat(imageMatToEncode);

      int requestedColorConversion = getColorConversion();
      if (requestedColorConversion >= 0) // if a color conversion is requested
      {
         // Convert color and assign to another mat to avoid changing the input data
         opencv_imgproc.cvtColor(image, tempMat, requestedColorConversion);
         image = tempMat;
      }

      // If the input and output dimensions or pixel formats don't match
      if (frameToEncode.width() != image.cols() || frameToEncode.height() != image.rows() || frameToScale.format() != frameToEncode.format())
      {
         if (swsContext == null) // Initialize frame scaling for first time
            initializeScaling(image.size());

         // Scale the frame and put data in the frame to encode
         frameToScale.data(0, image.data());
         error = sws_scale_frame(swsContext, frameToEncode, frameToScale);
         FFMPEGTools.checkNegativeError(error, "Scaling frame");
      }
      else // No scaling needed; put data directly into frame to encode
         frameToEncode.data(0, image.data());
   }

   private void initializeScaling(Size inputImageSize)
   {
      // Initialize an intermediate frame
      frameToScale.width(inputImageSize.width());
      frameToScale.height(inputImageSize.height());
      error = av_frame_get_buffer(frameToScale, 0);
      FFMPEGTools.checkNegativeError(error, "Getting input frame buffer");

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
      FFMPEGTools.checkPointer(swsContext, "Initializing swsContext");
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
