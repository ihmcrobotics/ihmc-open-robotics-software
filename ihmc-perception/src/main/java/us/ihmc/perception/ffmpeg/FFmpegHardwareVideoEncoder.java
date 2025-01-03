package us.ihmc.perception.ffmpeg;

import org.bytedeco.ffmpeg.avcodec.AVCodec;
import org.bytedeco.ffmpeg.avcodec.AVCodecHWConfig;
import org.bytedeco.ffmpeg.avformat.AVOutputFormat;
import org.bytedeco.ffmpeg.avutil.AVBufferRef;
import org.bytedeco.ffmpeg.avutil.AVComponentDescriptor;
import org.bytedeco.ffmpeg.avutil.AVDictionary;
import org.bytedeco.ffmpeg.avutil.AVHWFramesContext;
import org.bytedeco.ffmpeg.avutil.AVPixFmtDescriptor;
import org.bytedeco.javacpp.BytePointer;
import org.bytedeco.javacpp.Pointer;
import org.bytedeco.javacpp.SizeTPointer;
import org.bytedeco.opencv.global.opencv_cudaarithm;
import org.bytedeco.opencv.global.opencv_cudaimgproc;
import org.bytedeco.opencv.global.opencv_cudawarping;
import org.bytedeco.opencv.opencv_core.GpuMat;
import org.bytedeco.opencv.opencv_core.GpuMatVector;
import org.bytedeco.opencv.opencv_core.Size;
import us.ihmc.perception.RawImage;

import java.util.Objects;

import static org.bytedeco.cuda.global.cudart.*;
import static org.bytedeco.ffmpeg.global.avcodec.AV_CODEC_HW_CONFIG_METHOD_HW_FRAMES_CTX;
import static org.bytedeco.ffmpeg.global.avcodec.avcodec_get_hw_config;
import static org.bytedeco.ffmpeg.global.avutil.*;

/**
 * A video encoder that uses hardware (GPU) acceleration.
 */
public class FFmpegHardwareVideoEncoder extends FFmpegVideoEncoder
{
   private final AVCodecHWConfig hardwareConfiguration;
   private final AVBufferRef hardwareDeviceContext;
   private final AVBufferRef hardwareFramesReference;
   private final AVHWFramesContext hardwareFramesContext;

   private final Size outputSize;
   private final Size resizeTarget;
   private final BytePointer cudaPointer = new BytePointer();
   private final SizeTPointer cudaPitch = new SizeTPointer(1);

   private final int inputPixelFormat;
   private final AVPixFmtDescriptor inputPixelFormatDescriptor;

   /**
    * Creates a new video encoder. Must call {@link FFmpegEncoder#initialize(AVDictionary)} after this.
    * @param outputFormat The format of encoded output
    *                     (see: <a href="https://ffmpeg.org/ffmpeg-formats.html">FFmpeg Formats Documentation</a>,
    *                     or run {@code ffmpeg -hide_banner -formats} to see a list of formats).
    * @param preferredEncoderName Name of the encoder to use. Must have hardware acceleration support.
    *                             (run {@code ffmpeg -hide_banner -encoders} to see a list of encoders, though not all encoders may be available).
    * @param bitRate Target output bit rate.
    * @param outputWidth Width of the output video. If the output dimensions don't match that of the input, the input will be rescaled before encoding.
    * @param outputHeight Height of the output video. If the output dimensions don't match that of the input, the input will be rescaled before encoding.
    * @param groupOfPicturesSize Number of frames between key frames.
    * @param maxBFrames Maximum number of B frames in every GOP.
    * @param inputPixelFormat Pixel format of input images (one of avutil.AV_PIX_FMT_*).
    */
   public FFmpegHardwareVideoEncoder(AVOutputFormat outputFormat,
                                     String preferredEncoderName,
                                     int bitRate,
                                     int outputWidth,
                                     int outputHeight,
                                     int groupOfPicturesSize,
                                     int maxBFrames,
                                     int inputPixelFormat)
   {
      super(outputFormat, preferredEncoderName, bitRate, outputWidth, outputHeight, groupOfPicturesSize, maxBFrames);

      this.inputPixelFormat = inputPixelFormat;
      this.inputPixelFormatDescriptor = av_pix_fmt_desc_get(inputPixelFormat);

      outputSize = new Size(outputWidth, outputHeight);
      resizeTarget = new Size();

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
   public void setNextFrame(RawImage image)
   {
      setNextFrameAcquisitionTime(image.getAcquisitionTime().toEpochMilli());
      setNextFrame(image.getGpuImageMat());
   }

   @Override
   protected void prepareFrameForEncoding(Pointer gpuImageToEncode)
   {
      if (gpuImageToEncode instanceof GpuMat mat)
      {
         GpuMatVector matVector = new GpuMatVector();
         if (FFmpegTools.isPixelFormatPlanar(inputPixelFormat))
            opencv_cudaarithm.split(mat, matVector);
         else
            matVector.put(mat);

         prepareFrameForEncoding(matVector);
         matVector.close();
      }
      else if (gpuImageToEncode instanceof  GpuMatVector matVector)
         prepareFrameForEncoding(matVector);
   }

   private void prepareFrameForEncoding(GpuMatVector imagePlanes)
   {
      int planeCount = (int) imagePlanes.size();

      int requestedColorConversion = getColorConversion();
      if (requestedColorConversion >= 0) // if a color conversion is requested
      {
         // Convert color an assign to another gpu mat to avoid changing the input data
         for (int i = 0; i < planeCount; ++i)
         {
            GpuMat plane = imagePlanes.get(i);
            opencv_cudaimgproc.cvtColor(plane, plane, requestedColorConversion);
         }
      }

      // Get the first plane (either luma plane, or the whole image if it's not planar)
      GpuMat firstPlane = imagePlanes.get(0);
      int width = firstPlane.cols();
      int height = firstPlane.rows();
      long pitch = firstPlane.step();

      // If the input and output dimensions don't match
      if (outputSize.width() != width || outputSize.height() != height)
      {  // Find the scale factor
         double widthScaleFactor = (double) outputSize.width() / width;
         double heightScaleFactor = (double) outputSize.height() / height;

         // Rescale all planes by that factor
         for (int i = 0; i < planeCount; ++i)
         {
            // Resize and put data in the frame to encode
            GpuMat plane = imagePlanes.get(i);
            resizeTarget.width((int) (plane.cols() * widthScaleFactor));
            resizeTarget.height((int) (plane.rows() * heightScaleFactor));
            opencv_cudawarping.resize(plane, plane, resizeTarget);
         }

         // Update the first plane dimensions
         width = firstPlane.cols();
         height = firstPlane.rows();
         pitch = firstPlane.step();
      }

      // Ensure we have CUDA memory for the data
      if (cudaPointer.isNull())
      {
         allocateCUDAMemory(width, height, inputPixelFormat);
         frameToEncode.data(0, cudaPointer);
         frameToEncode.linesize(0, (int) cudaPitch.get());
      }

      // Copy the 1st plane into the frame's CUDA memory
      AVComponentDescriptor firstPlaneComponent = inputPixelFormatDescriptor.comp(0);
      int widthInBytes = (firstPlaneComponent.step() * firstPlaneComponent.depth() * width) / 8;
      cudaMemcpy2D(cudaPointer, cudaPitch.get(), firstPlane.data(), pitch, widthInBytes, height, cudaMemcpyDefault);
      firstPlaneComponent.close();

      // Copy chroma planes into CUDA memory
      long offset = height * pitch;
      for (int i = 1; i < planeCount; ++i)
      {
         GpuMat plane = imagePlanes.get(i);
         Pointer chromaPointer = cudaPointer.getPointer(offset);
         int chromaPitch = FFmpegTools.getChromaWidth((int) cudaPitch.get(), inputPixelFormat);
         cudaMemcpy2D(chromaPointer, chromaPitch, plane.data(), plane.step(), plane.cols(), plane.rows(), cudaMemcpyDefault);
         offset += (long) plane.rows() * chromaPitch;
      }
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

   private void allocateCUDAMemory(int width, int height, int pixelFormat)
   {
      int allocationWidth = width;
      int allocationHeight = height;

      for (int i = 0; i < inputPixelFormatDescriptor.nb_components(); ++i)
      {
         AVComponentDescriptor component = inputPixelFormatDescriptor.comp(i);
         if (component.plane() == 0)
            allocationWidth = (component.step() * component.depth() * width) / 8;
         else
            allocationHeight += FFmpegTools.getChromaHeight(height, pixelFormat);
         component.close();
      }

      cudaMallocPitch(cudaPointer, cudaPitch, allocationWidth, allocationHeight);
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

      cudaFree(cudaPointer);
      cudaPointer.close();
      cudaPitch.close();

      outputSize.close();
      resizeTarget.close();
      inputPixelFormatDescriptor.close();
   }
}
