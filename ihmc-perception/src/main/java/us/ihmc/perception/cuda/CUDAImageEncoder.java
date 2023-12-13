package us.ihmc.perception.cuda;

import org.bytedeco.cuda.cudart.CUctx_st;
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

/**
 * Used for encoding images using CUDA.
 * Ensure that the computer has CUDA available before using this class. Most all Nvidia graphics cards should have CUDA available.
 * Easy check whether computer has an Nvidia graphics card:
 *    try
 *    {
 *       ProcessTools.execSimpleCommand("nvidia-smi");
 *       hasNvidiaGPU = true;
 *    }
 *    catch (IOException | InterruptedException ignored)
 *    {
 *       hasNvidiaGPU = false;
 *    }
 *
 * This class has been adapted from the SampleJpegEncoder of bytedeco:
 *    https://github.com/bytedeco/javacpp-presets/blob/master/cuda/samples/SampleJpegEncoder.java
 */
public class CUDAImageEncoder
{
   private final CUctx_st cudaContext;
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
      // Initialize context
      cudaContext = new CUctx_st();
      checkCUDA("cuInit", cuInit(0));
      checkCUDA("cuCtxCreate", cuCtxCreate(cudaContext, CU_CTX_SCHED_BLOCKING_SYNC, 0));

      // Initialize stream
      cudaStream = new CUstream_st();
      checkCUDA("cuStreamCreate", cuStreamCreate(cudaStream, NVJPEG_FLAGS_DEFAULT));

      // Initialize handle
      nvjpegHandle = new nvjpegHandle();
      checkNVJPEG("nvjpegCreateSimple", nvjpegCreateSimple(nvjpegHandle));

      // Initialize encoder state
      encoderState = new nvjpegEncoderState();
      checkNVJPEG("nvjpegEncoderStateCreate", nvjpegEncoderStateCreate(nvjpegHandle, encoderState, cudaStream));

      // Initialize encoder parameters
      encoderParameters = new nvjpegEncoderParams();
      checkNVJPEG("nvjpegEncoderParamsCreate", nvjpegEncoderParamsCreate(nvjpegHandle, encoderParameters, cudaStream));

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

      checkCUDA("cuCtxSetCurrent", cuCtxSetCurrent(cudaContext));

      // Set params to correct sampling factor
      checkNVJPEG("nvjpegEncoderParamsSetSamplingFactors", nvjpegEncoderParamsSetSamplingFactors(encoderParameters, NVJPEG_CSS_420, cudaStream));

      // Get Y plane data
      BytePointer yPlanePointer = new BytePointer(); // create a pointer for the Y plane
      checkCUDA("cudaMalloc", cudaMalloc(yPlanePointer, frameSize)); // allocate Y plane memory
      checkCUDA("cudaMemcpy", cudaMemcpy(yPlanePointer, sourceImage, frameSize, cudaMemcpyHostToDevice)); // copy Y plane data to device memory
      nvjpegImage.pitch(0, imageWidth); // set the pitch
      nvjpegImage.channel(0, yPlanePointer); //set the channel
      sourceImage.position(sourceImage.position() + frameSize); // move pointer to start of U plane

      // Get U plane data
      BytePointer uPlanePointer = new BytePointer();
      checkCUDA("cudaMalloc", cudaMalloc(uPlanePointer, quarterOfFrameSize));
      checkCUDA("cudaMemcpy", cudaMemcpy(uPlanePointer, sourceImage, quarterOfFrameSize, cudaMemcpyHostToDevice));
      nvjpegImage.pitch(1, halfOfImageWidth);
      nvjpegImage.channel(1, uPlanePointer);
      sourceImage.position(sourceImage.position() + quarterOfFrameSize);

      // Get V plane data
      BytePointer vPlanePointer = new BytePointer();
      checkCUDA("cudaMalloc", cudaMalloc(vPlanePointer, quarterOfFrameSize));
      checkCUDA("cudaMemcpy", cudaMemcpy(vPlanePointer, sourceImage, quarterOfFrameSize, cudaMemcpyHostToDevice));
      nvjpegImage.pitch(2, halfOfImageWidth);
      nvjpegImage.channel(2, vPlanePointer);

      // Encode image (mem)
      checkNVJPEG("nvjpegEncodeYUV",
                  nvjpegEncodeYUV(nvjpegHandle, encoderState, encoderParameters, nvjpegImage, NVJPEG_CSS_420, imageWidth, imageHeight, cudaStream));

      // Get compressed size
      SizeTPointer jpegSize = new SizeTPointer(1);
      checkNVJPEG("nvjpegEncodeRetrieveBitstream", nvjpegEncodeRetrieveBitstream(nvjpegHandle, encoderState, (BytePointer) null, jpegSize, cudaStream));

      // Retrieve bitstream
      BytePointer jpegBytePointer = new BytePointer(jpegSize.get());
      checkNVJPEG("nvjpegEncodeRetrieveBitstream", nvjpegEncodeRetrieveBitstream(nvjpegHandle, encoderState, jpegBytePointer, jpegSize, cudaStream));

      // Synchronize cuda stream
      checkCUDA("cudaStreamSynchronize", cudaStreamSynchronize(cudaStream));

      //copy data into output pointer
      byte[] bytes = new byte[(int) jpegBytePointer.limit()];
      jpegBytePointer.get(bytes, 0, (int) jpegBytePointer.limit());
      outputImagePointer.put(bytes, 0, (int) jpegBytePointer.limit());
      outputImagePointer.limit(jpegBytePointer.limit());

      // Free GPU memory
      checkCUDA("cudaFree", cudaFree(yPlanePointer));
      checkCUDA("cudaFree", cudaFree(uPlanePointer));
      checkCUDA("cudaFree", cudaFree(vPlanePointer));

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

      checkCUDA("cuCtxSetCurrent", cuCtxSetCurrent(cudaContext));

      // Set params to correct sampling factor
      checkNVJPEG("nvjpegEncoderParamsSetSamplingFactors", nvjpegEncoderParamsSetSamplingFactors(encoderParameters, NVJPEG_CSS_444, cudaStream));

      // Get B plane data
      BytePointer devicePointer = new BytePointer();
      checkCUDA("cudaMalloc", cudaMalloc(devicePointer, frameSize)); // allocate GPU memory
      checkCUDA("cudaMemcpy2D", cudaMemcpy2D(devicePointer, rowSize, sourceImage, sourceImagePitch, rowSize, imageHeight, cudaMemcpyDeviceToDevice));
      nvjpegImage.pitch(0, rowSize);
      nvjpegImage.channel(0, devicePointer);

      // Encode the image
      checkNVJPEG("nvjpegEncodeImage",
                  nvjpegEncodeImage(nvjpegHandle, encoderState, encoderParameters, nvjpegImage, NVJPEG_INPUT_BGRI, imageWidth, imageHeight, cudaStream));

      // Get compressed size
      SizeTPointer jpegSize = new SizeTPointer(1);
      checkNVJPEG("nvjpegEncodeRetrieveBitstream", nvjpegEncodeRetrieveBitstream(nvjpegHandle, encoderState, (BytePointer) null, jpegSize, cudaStream));

      // Retrieve bitstream
      BytePointer jpegBytePointer = new BytePointer(jpegSize.get());
      checkNVJPEG("nvjpegEncodeRetrieveBitstream", nvjpegEncodeRetrieveBitstream(nvjpegHandle, encoderState, jpegBytePointer, jpegSize, cudaStream));

      // Synchronize cuda stream
      checkCUDA("cudaStreamSynchronize", cudaStreamSynchronize(cudaStream));

      //copy data into output pointer
      byte[] bytes = new byte[(int) jpegBytePointer.limit()];
      jpegBytePointer.get(bytes, 0, (int) jpegBytePointer.limit());
      outputImagePointer.put(bytes, 0, (int) jpegBytePointer.limit());
      outputImagePointer.limit(jpegBytePointer.limit());

      // Free GPU memory
      checkCUDA("cudaFree", cudaFree(devicePointer));

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

      checkCUDA("cuCtxSetCurrent", cuCtxSetCurrent(cudaContext));

      // Set params to correct sampling factor
      checkNVJPEG("nvjpegEncoderParamsSetSamplingFactors", nvjpegEncoderParamsSetSamplingFactors(encoderParameters, NVJPEG_CSS_444, cudaStream));

      // Get B plane data
      BytePointer devicePointer = new BytePointer();
      checkCUDA("cudaMalloc", cudaMalloc(devicePointer, frameSize)); // allocate GPU memory
      checkCUDA("cudaMemcpy2D", cudaMemcpy2D(devicePointer, rowSize, sourceImage, sourceImagePitch, rowSize, imageHeight, cudaMemcpyDeviceToDevice));
      nvjpegImage.pitch(0, rowSize);
      nvjpegImage.channel(0, devicePointer);

      // Encode the image
      checkNVJPEG("nvjpegEncodeImage",
                  nvjpegEncodeImage(nvjpegHandle, encoderState, encoderParameters, nvjpegImage, NVJPEG_INPUT_RGBI, imageWidth, imageHeight, cudaStream));

      // Get compressed size
      SizeTPointer jpegSize = new SizeTPointer(1);
      checkNVJPEG("nvjpegEncodeRetrieveBitstream", nvjpegEncodeRetrieveBitstream(nvjpegHandle, encoderState, (BytePointer) null, jpegSize, cudaStream));

      // Retrieve bitstream
      BytePointer jpegBytePointer = new BytePointer(jpegSize.get());
      checkNVJPEG("nvjpegEncodeRetrieveBitstream", nvjpegEncodeRetrieveBitstream(nvjpegHandle, encoderState, jpegBytePointer, jpegSize, cudaStream));

      // Synchronize cuda stream
      checkCUDA("cudaStreamSynchronize", cudaStreamSynchronize(cudaStream));

      //copy data into output pointer
      byte[] bytes = new byte[(int) jpegBytePointer.limit()];
      jpegBytePointer.get(bytes, 0, (int) jpegBytePointer.limit());
      outputImagePointer.put(bytes, 0, (int) jpegBytePointer.limit());
      outputImagePointer.limit(jpegBytePointer.limit());

      // Free GPU memory
      checkCUDA("cudaFree", cudaFree(devicePointer));

      // Close pointers
      jpegSize.close();
      jpegBytePointer.close();
   }

   public void destroy()
   {
      checkNVJPEG("nvjpegEncoderParamsDestroy", nvjpegEncoderParamsDestroy(encoderParameters));
      checkNVJPEG("nvjpegEncoderStateDestroy", nvjpegEncoderStateDestroy(encoderState));
      checkNVJPEG("nvjpegDestroy", nvjpegDestroy(nvjpegHandle));
      checkCUDA("cuStreamDestroy", cuStreamDestroy(cudaStream));
      checkCUDA("cuCtxDestroy", cuCtxDestroy(cudaContext));
   }

   /**
    * Helper function for CUDA error checking.
    * Example use:
    *    CHECK_CUDA("cudaFree", cudaFree(devicePointer));
    * @param functionName the name of the function being called as a string
    * @param result the returned error code from the function called. A CUDA function should be called for this parameter
    */
   private static void checkCUDA(String functionName, int result)
   {
      if (result != CUDA_SUCCESS)
      {
         LogTools.error(String.format("%s returned '%d'", functionName, result));
      }
   }

   /**
    * Helper function for nvjpeg error checking.
    * Example use:
    *    CHECK_NVJPEG("nvjpegCreateSimple", nvjpegCreateSimple(handle));
    * @param functionName the name of the function being called as a string
    * @param result the returned error code from the function called. An nvjpeg function should be called for this parameter
    */
   private static void checkNVJPEG(String functionName, int result)
   {
      if (result != NVJPEG_STATUS_SUCCESS)
      {
         LogTools.error(String.format("%s returned '%d'", functionName, result));
      }
   }
}
