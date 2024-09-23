package us.ihmc.perception.ffmpeg;

import org.bytedeco.ffmpeg.avcodec.AVCodec;
import org.bytedeco.ffmpeg.avcodec.AVCodecHWConfig;
import org.bytedeco.ffmpeg.avformat.AVOutputFormat;
import org.bytedeco.ffmpeg.avutil.AVBufferRef;
import org.bytedeco.ffmpeg.avutil.AVHWFramesContext;
import org.bytedeco.javacpp.Pointer;
import org.bytedeco.opencv.global.opencv_cudaimgproc;
import org.bytedeco.opencv.global.opencv_cudawarping;
import org.bytedeco.opencv.opencv_core.GpuMat;
import org.bytedeco.opencv.opencv_core.Size;

import java.util.Objects;

import static org.bytedeco.ffmpeg.global.avcodec.AV_CODEC_HW_CONFIG_METHOD_HW_FRAMES_CTX;
import static org.bytedeco.ffmpeg.global.avcodec.avcodec_get_hw_config;
import static org.bytedeco.ffmpeg.global.avutil.*;

public class FFmpegHardwareVideoEncoder extends FFmpegVideoEncoder
{
   private final AVCodecHWConfig hardwareConfiguration;
   private final AVBufferRef hardwareDeviceContext;
   private final AVBufferRef hardwareFramesReference;
   private final AVHWFramesContext hardwareFramesContext;

   private final Size outputSize;
   private final GpuMat tempGpuMat;

   public FFmpegHardwareVideoEncoder(AVOutputFormat outputFormat,
                                     String preferredCodecName,
                                     int bitRate,
                                     int outputWidth,
                                     int outputHeight,
                                     int groupOfPicturesSize,
                                     int maxBFrames,
                                     int inputPixelFormat)
   {
      super(outputFormat, preferredCodecName, bitRate, outputWidth, outputHeight, groupOfPicturesSize, maxBFrames);

      outputSize = new Size(outputWidth, outputHeight);
      tempGpuMat = new GpuMat();

      // Find the hardware configuration for the encoder
      hardwareConfiguration = getCodecHardwareConfiguration(encoder);
      FFmpegTools.checkPointer(hardwareConfiguration, "Finding encoder hardware configuration.");
      Objects.requireNonNull(hardwareConfiguration); // just here to silence a warning, even though checkPointer does the job :(

      // Create hardware related contexts
      hardwareDeviceContext = new AVBufferRef();
      error = av_hwdevice_ctx_create(hardwareDeviceContext, hardwareConfiguration.device_type(), (String) null, null, 0);

      hardwareFramesReference = av_hwframe_ctx_alloc(hardwareDeviceContext);
      FFmpegTools.checkPointer(hardwareFramesReference, "Creating frame context");

      hardwareFramesContext = new AVHWFramesContext(hardwareFramesReference.data());
      hardwareFramesContext.format(hardwareConfiguration.pix_fmt());
      hardwareFramesContext.sw_format(inputPixelFormat);
      hardwareFramesContext.width(outputWidth);
      hardwareFramesContext.height(outputHeight);
      hardwareFramesContext.initial_pool_size(2 * groupOfPicturesSize);
      error = av_hwframe_ctx_init(hardwareFramesReference);
      FFmpegTools.checkNegativeError(error, "Initializing frame context");

      // Set the frame to be encoded accordingly
      frameToEncode.format(hardwareConfiguration.pix_fmt());
      error = av_hwframe_get_buffer(hardwareFramesReference, frameToEncode, 0);
      FFmpegTools.checkError(error, frameToEncode.hw_frames_ctx(), "Getting next frame buffer");

      // Set encoder parameters accordingly
      encoderContext.pix_fmt(hardwareConfiguration.pix_fmt());
      encoderContext.hw_frames_ctx(av_buffer_ref(hardwareFramesReference));
      FFmpegTools.checkPointer(encoderContext.hw_frames_ctx(), "Allocating hardware frames");

      av_buffer_unref(hardwareFramesReference);
   }

   @Override
   protected void prepareFrameForEncoding(Pointer imageGpuMatToEncode)
   {
      GpuMat image = new GpuMat(imageGpuMatToEncode);

      int requestedColorConversion = getColorConversion();
      if (requestedColorConversion >= 0) // if a color conversion is requested
      {
         // Convert color an assign to another gpu mat to avoid changing the input data
         opencv_cudaimgproc.cvtColor(image, tempGpuMat, requestedColorConversion);
         image = tempGpuMat;
      }

      // If the input and output dimensions don't match
      if (outputSize.width() != image.cols() || outputSize.height() != image.rows())
      {
         // Resize and put data in the frame to encode
         opencv_cudawarping.resize(image, tempGpuMat, outputSize);
         frameToEncode.data(0, tempGpuMat.data());
      }
      else // No resizing needed; put data directly into frame to encode
         frameToEncode.data(0, image.data());
   }

   private AVCodecHWConfig getCodecHardwareConfiguration(AVCodec encoder)
   {
      AVCodecHWConfig hardwareConfig;
      for (int i = 0; ; i++)
      {
         hardwareConfig = avcodec_get_hw_config(encoder, i);
         if (hardwareConfig == null || hardwareConfig.isNull())
            return null;

         if ((hardwareConfig.methods() & AV_CODEC_HW_CONFIG_METHOD_HW_FRAMES_CTX) != 0)
            return hardwareConfig;

         hardwareConfig.close();
      }
   }

   @Override
   public void destroy()
   {
      super.destroy();

      av_buffer_unref(hardwareDeviceContext);
      hardwareDeviceContext.close();
      hardwareFramesContext.close();
      hardwareFramesReference.close();
      hardwareConfiguration.close();

      outputSize.close();
      tempGpuMat.close();
   }
}
