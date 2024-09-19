package us.ihmc.perception.ffmpeg;

import org.bytedeco.ffmpeg.avcodec.AVCodec;
import org.bytedeco.ffmpeg.avcodec.AVCodecHWConfig;
import org.bytedeco.ffmpeg.avformat.AVOutputFormat;
import org.bytedeco.ffmpeg.avutil.AVBufferRef;
import org.bytedeco.ffmpeg.avutil.AVDictionary;
import org.bytedeco.ffmpeg.avutil.AVHWFramesContext;
import org.bytedeco.opencv.opencv_core.GpuMat;
import org.bytedeco.opencv.opencv_core.Mat;

import static org.bytedeco.ffmpeg.global.avcodec.AV_CODEC_HW_CONFIG_METHOD_HW_DEVICE_CTX;
import static org.bytedeco.ffmpeg.global.avcodec.avcodec_get_hw_config;
import static org.bytedeco.ffmpeg.global.avutil.*;

public class FFMPEGHardwareVideoEncoder extends FFMPEGVideoEncoder
{
   private final AVCodecHWConfig hardwareConfiguration;
   private final AVBufferRef hardwareDeviceContext;
   private final AVBufferRef hardwareFramesReference;
   private final AVHWFramesContext hardwareFramesContext;

   public FFMPEGHardwareVideoEncoder(AVOutputFormat outputFormat,
                                     String preferredCodecName,
                                     int bitRate,
                                     int outputWidth,
                                     int outputHeight,
                                     int outputPixelFormat,
                                     int groupOfPicturesSize,
                                     int maxBFrames,
                                     int inputPixelFormat)
   {
      super(outputFormat, preferredCodecName, bitRate, outputWidth, outputHeight, outputPixelFormat, groupOfPicturesSize, maxBFrames, inputPixelFormat);

      hardwareConfiguration = getCodecHardwareConfiguration(encoder);
      FFMPEGTools.checkPointer(hardwareConfiguration, "Finding encoder hardware configuration.");

      hardwareDeviceContext = new AVBufferRef();
      error = av_hwdevice_ctx_create(hardwareDeviceContext, hardwareConfiguration.device_type(), (String) null, null, 0);

      hardwareFramesReference = av_hwframe_ctx_alloc(hardwareDeviceContext);
      FFMPEGTools.checkPointer(hardwareFramesReference, "Creating frame context");

      hardwareFramesContext = new AVHWFramesContext(hardwareFramesReference.data());
      hardwareFramesContext.format(hardwareConfiguration.pix_fmt());
      hardwareFramesContext.sw_format(outputPixelFormat);
      hardwareFramesContext.width(outputWidth);
      hardwareFramesContext.height(outputHeight);
      hardwareFramesContext.initial_pool_size(3 * groupOfPicturesSize);
      error = av_hwframe_ctx_init(hardwareFramesReference);
      FFMPEGTools.checkNegativeError(error, "Initializing frame context");

      encoderContext.hw_device_ctx(av_buffer_ref(hardwareDeviceContext));
      encoderContext.pix_fmt(hardwareConfiguration.pix_fmt());
      encoderContext.hw_device_ctx(hardwareDeviceContext);
      encoderContext.hw_frames_ctx(av_buffer_ref(hardwareFramesReference));
      FFMPEGTools.checkPointer(encoderContext.hw_frames_ctx(), "Allocating hardware frames");

      av_buffer_unref(hardwareFramesReference);
   }

   @Override
   public void setNextFrame(Mat frame)
   {
      GpuMat gpuFrame = new GpuMat();
      gpuFrame.upload(frame);
      setNextFrame(gpuFrame);
   }

   public void setNextFrame(GpuMat frame)
   {
      // TODO: Finish
      // TODO: Find out whether sws_scale can work on GPU
   }

   private AVCodecHWConfig getCodecHardwareConfiguration(AVCodec encoder)
   {
      AVCodecHWConfig hardwareConfig;
      for (int i = 0; ; i++)
      {
         hardwareConfig = avcodec_get_hw_config(encoder, i);
         if (hardwareConfig == null || hardwareConfig.isNull())
            return null;

         if ((hardwareConfig.methods() & AV_CODEC_HW_CONFIG_METHOD_HW_DEVICE_CTX) != 0)
            return hardwareConfig;

         hardwareConfig.close();
      }
   }
}
