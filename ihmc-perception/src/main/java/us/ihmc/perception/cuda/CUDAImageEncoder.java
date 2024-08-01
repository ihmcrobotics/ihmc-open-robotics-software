package us.ihmc.perception.cuda;

import org.bytedeco.cuda.cudart.CUstream_st;
import org.bytedeco.cuda.nvjpeg.nvjpegEncoderParams;
import org.bytedeco.cuda.nvjpeg.nvjpegEncoderState;
import org.bytedeco.cuda.nvjpeg.nvjpegHandle;
import org.bytedeco.cuda.nvjpeg.nvjpegImage_t;
import org.bytedeco.javacpp.BytePointer;
import org.bytedeco.javacpp.SizeTPointer;
import us.ihmc.log.LogTools;

import static org.bytedeco.cuda.global.cudart.*;
import static org.bytedeco.cuda.global.nvjpeg.*;
import static us.ihmc.perception.cuda.CUDATools.checkCUDAError;

/**
 * Used for encoding images using CUDA.
 * Ensure that the computer has CUDA available before using this class. Most all Nvidia graphics cards should have CUDA available.
 *
 * This class has been adapted from the SampleJpegEncoder of bytedeco:
 *    https://github.com/bytedeco/javacpp-presets/blob/master/cuda/samples/SampleJpegEncoder.java
 */
public class CUDAImageEncoder
{
   private final CUstream_st cudaStream;
   private final nvjpegHandle nvjpegHandle;
   private final nvjpegEncoderState encoderState;
   private final nvjpegEncoderParams encoderParameters;
   private final nvjpegImage_t nvjpegImage;

   /**
    * Initialize necessary CUDA components
    */
   public CUDAImageEncoder()
   {
      // Initialize stream
      cudaStream = CUDAStreamManager.getStream();

      // Initialize handle
      nvjpegHandle = new nvjpegHandle();
      checkNVJPEG(nvjpegCreateSimple(nvjpegHandle));

      // Initialize encoder state
      encoderState = new nvjpegEncoderState();
      checkNVJPEG(nvjpegEncoderStateCreate(nvjpegHandle, encoderState, cudaStream));

      // Initialize encoder parameters
      encoderParameters = new nvjpegEncoderParams();
      checkNVJPEG(nvjpegEncoderParamsCreate(nvjpegHandle, encoderParameters, cudaStream));

      // Initialize nvjpeg image
      nvjpegImage = new nvjpegImage_t();
   }

   /**
    * Encodes a YUV I420 image into jpeg.
    * @param sourceImage the YUV I420 image
    * @param outputImagePointer pointer to the jpeg image output
    * @param imageWidth width (in bytes) of the source image
    * @param imageHeight height (in bytes) of the source image
    */
   public void encodeYUV420(BytePointer sourceImage, BytePointer outputImagePointer, int imageWidth, int imageHeight)
   {
      int frameSize = imageWidth * imageHeight;
      long quarterOfFrameSize = ((frameSize % 4 == 0) ? (frameSize / 4) : (frameSize / 4 + 1)); // ensure integer math goes well
      long halfOfImageWidth = ((imageWidth % 2 == 0) ? (imageWidth / 2) : (imageWidth / 2 + 1));

      // Set params to correct sampling factor
      checkNVJPEG(nvjpegEncoderParamsSetSamplingFactors(encoderParameters, NVJPEG_CSS_420, cudaStream));

      // Get Y plane data
      BytePointer yPlanePointer = new BytePointer(); // create a pointer for the Y plane
      checkCUDAError(cudaMalloc(yPlanePointer, frameSize)); // allocate Y plane memory
      checkCUDAError(cudaMemcpy(yPlanePointer, sourceImage, frameSize, cudaMemcpyHostToDevice)); // copy Y plane data to device memory
      nvjpegImage.pitch(0, imageWidth); // set the pitch
      nvjpegImage.channel(0, yPlanePointer); //set the channel
      sourceImage.position(sourceImage.position() + frameSize); // move pointer to start of U plane

      // Get U plane data
      BytePointer uPlanePointer = new BytePointer();
      checkCUDAError(cudaMalloc(uPlanePointer, quarterOfFrameSize));
      checkCUDAError(cudaMemcpy(uPlanePointer, sourceImage, quarterOfFrameSize, cudaMemcpyHostToDevice));
      nvjpegImage.pitch(1, halfOfImageWidth);
      nvjpegImage.channel(1, uPlanePointer);
      sourceImage.position(sourceImage.position() + quarterOfFrameSize);

      // Get V plane data
      BytePointer vPlanePointer = new BytePointer();
      checkCUDAError(cudaMalloc(vPlanePointer, quarterOfFrameSize));
      checkCUDAError(cudaMemcpy(vPlanePointer, sourceImage, quarterOfFrameSize, cudaMemcpyHostToDevice));
      nvjpegImage.pitch(2, halfOfImageWidth);
      nvjpegImage.channel(2, vPlanePointer);

      // Encode image (mem)
      checkNVJPEG(nvjpegEncodeYUV(nvjpegHandle, encoderState, encoderParameters, nvjpegImage, NVJPEG_CSS_420, imageWidth, imageHeight, cudaStream));

      // Get compressed size
      SizeTPointer jpegSize = new SizeTPointer(1);
      checkNVJPEG(nvjpegEncodeRetrieveBitstream(nvjpegHandle, encoderState, (BytePointer) null, jpegSize, cudaStream));

      // Retrieve bitstream
      BytePointer jpegBytePointer = new BytePointer(jpegSize.get());
      checkNVJPEG(nvjpegEncodeRetrieveBitstream(nvjpegHandle, encoderState, jpegBytePointer, jpegSize, cudaStream));

      // Synchronize cuda stream
      checkCUDAError(cudaStreamSynchronize(cudaStream));

      //copy data into output pointer
      byte[] bytes = new byte[(int) jpegBytePointer.limit()];
      jpegBytePointer.get(bytes, 0, (int) jpegBytePointer.limit());
      outputImagePointer.put(bytes, 0, (int) jpegBytePointer.limit());
      outputImagePointer.limit(jpegBytePointer.limit());

      // Free GPU memory
      checkCUDAError(cudaFree(yPlanePointer));
      checkCUDAError(cudaFree(uPlanePointer));
      checkCUDAError(cudaFree(vPlanePointer));

      // Close pointers
      yPlanePointer.close();
      uPlanePointer.close();
      vPlanePointer.close();
      jpegSize.close();
   }

   /**
    * Encodes a BGR image into jpeg
    * @param sourceImage the BGR image
    * @param outputImagePointer pointer to the jpeg image output
    * @param imageWidth width (in bytes) of the source image
    * @param imageHeight height (in bytes) of the source image
    * @param sourceImagePitch number of bytes (including padding) in a row of the image
    */
   public void encodeBGR(BytePointer sourceImage, BytePointer outputImagePointer, int imageWidth, int imageHeight, long sourceImagePitch)
   {
      long rowSize = 3L * imageWidth;
      long frameSize = 3L * imageWidth * imageHeight;

      // Set params to correct sampling factor
      checkNVJPEG(nvjpegEncoderParamsSetSamplingFactors(encoderParameters, NVJPEG_CSS_444, cudaStream));

      // Get B plane data
      BytePointer devicePointer = new BytePointer();
      checkCUDAError(cudaMalloc(devicePointer, frameSize)); // allocate GPU memory
      checkCUDAError(cudaMemcpy2D(devicePointer, rowSize, sourceImage, sourceImagePitch, rowSize, imageHeight, cudaMemcpyDeviceToDevice));
      nvjpegImage.pitch(0, rowSize);
      nvjpegImage.channel(0, devicePointer);

      // Encode the image
      checkNVJPEG(nvjpegEncodeImage(nvjpegHandle, encoderState, encoderParameters, nvjpegImage, NVJPEG_INPUT_BGRI, imageWidth, imageHeight, cudaStream));

      // Get compressed size
      SizeTPointer jpegSize = new SizeTPointer(1);
      checkNVJPEG(nvjpegEncodeRetrieveBitstream(nvjpegHandle, encoderState, (BytePointer) null, jpegSize, cudaStream));

      // Retrieve bitstream
      BytePointer jpegBytePointer = new BytePointer(jpegSize.get());
      checkNVJPEG(nvjpegEncodeRetrieveBitstream(nvjpegHandle, encoderState, jpegBytePointer, jpegSize, cudaStream));

      // Synchronize cuda stream
      checkCUDAError(cudaStreamSynchronize(cudaStream));

      //copy data into output pointer
      byte[] bytes = new byte[(int) jpegBytePointer.limit()];
      jpegBytePointer.get(bytes, 0, (int) jpegBytePointer.limit());
      outputImagePointer.put(bytes, 0, (int) jpegBytePointer.limit());
      outputImagePointer.limit(jpegBytePointer.limit());

      // Free GPU memory
      checkCUDAError(cudaFree(devicePointer));

      // Close pointers
      jpegSize.close();
      jpegBytePointer.close();
   }

   /**
    * Despite encoding an RGB image, the returned JPEG seems to be in BGR format.
    *
    * @param sourceImage the RGB image
    * @param outputImagePointer pointer to the jpeg image output
    * @param imageWidth width (in bytes) of the source image
    * @param imageHeight height (in bytes) of the source image
    * @param sourceImagePitch number of bytes (including padding) in a row of the image
    */
   @Deprecated
   public void encodeRGB(BytePointer sourceImage, BytePointer outputImagePointer, int imageWidth, int imageHeight, long sourceImagePitch)
   {
      long rowSize = 3L * imageWidth;
      long frameSize = 3L * imageWidth * imageHeight;

      // Set params to correct sampling factor
      checkNVJPEG(nvjpegEncoderParamsSetSamplingFactors(encoderParameters, NVJPEG_CSS_444, cudaStream));

      // Get B plane data
      BytePointer devicePointer = new BytePointer();
      checkCUDAError(cudaMalloc(devicePointer, frameSize)); // allocate GPU memory
      checkCUDAError(cudaMemcpy2D(devicePointer, rowSize, sourceImage, sourceImagePitch, rowSize, imageHeight, cudaMemcpyDeviceToDevice));
      nvjpegImage.pitch(0, rowSize);
      nvjpegImage.channel(0, devicePointer);

      // Encode the image
      checkNVJPEG(nvjpegEncodeImage(nvjpegHandle, encoderState, encoderParameters, nvjpegImage, NVJPEG_INPUT_RGBI, imageWidth, imageHeight, cudaStream));

      // Get compressed size
      SizeTPointer jpegSize = new SizeTPointer(1);
      checkNVJPEG(nvjpegEncodeRetrieveBitstream(nvjpegHandle, encoderState, (BytePointer) null, jpegSize, cudaStream));

      // Retrieve bitstream
      BytePointer jpegBytePointer = new BytePointer(jpegSize.get());
      checkNVJPEG(nvjpegEncodeRetrieveBitstream(nvjpegHandle, encoderState, jpegBytePointer, jpegSize, cudaStream));

      // Synchronize cuda stream
      checkCUDAError(cudaStreamSynchronize(cudaStream));

      //copy data into output pointer
      byte[] bytes = new byte[(int) jpegBytePointer.limit()];
      jpegBytePointer.get(bytes, 0, (int) jpegBytePointer.limit());
      outputImagePointer.put(bytes, 0, (int) jpegBytePointer.limit());
      outputImagePointer.limit(jpegBytePointer.limit());

      // Free GPU memory
      checkCUDAError(cudaFree(devicePointer));

      // Close pointers
      jpegSize.close();
      jpegBytePointer.close();
   }

   public void destroy()
   {
      checkNVJPEG(nvjpegEncoderParamsDestroy(encoderParameters));
      checkNVJPEG(nvjpegEncoderStateDestroy(encoderState));
      checkNVJPEG(nvjpegDestroy(nvjpegHandle));
      checkCUDAError(cuStreamDestroy(cudaStream));
   }

   /**
    * Helper function for nvjpeg error checking.
    * Example use:
    *    CHECK_NVJPEG(nvjpegCreateSimple(handle));
    * @param result the returned error code from the function called. An nvjpeg function should be called for this parameter
    */
   private static void checkNVJPEG(int result)
   {
      if (result != NVJPEG_STATUS_SUCCESS)
      {
         String errorName = switch (result)
         {
            case 1 -> "NVJPEG_STATUS_NOT_INITIALIZED";
            case 2 -> "NVJPEG_STATUS_INVALID_PARAMETER";
            case 3 -> "NVJPEG_STATUS_BAD_JPEG";
            case 4 -> "NVJPEG_STATUS_JPEG_NOT_SUPPORTED";
            case 5 -> "NVJPEG_STATUS_ALLOCATOR_FAILURE";
            case 6 -> "NVJPEG_STATUS_EXECUTION_FAILED";
            case 7 -> "NVJPEG_STATUS_ARCH_MISMATCH";
            case 8 -> "NVJPEG_STATUS_INTERNAL_ERROR";
            case 9 -> "NVJPEG_STATUS_IMPLEMENTATION_NOT_SUPPORTED";
            case 10 -> "NVJPEG_STATUS_INCOMPLETE_BITSTREAM";
            default -> "UNKNOWN";
         };
         LogTools.error("NVJPEG Error ({}): {}", result, errorName);
      }
   }
}
