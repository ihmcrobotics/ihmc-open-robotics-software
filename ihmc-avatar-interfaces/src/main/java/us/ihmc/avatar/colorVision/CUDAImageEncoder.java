package us.ihmc.avatar.colorVision;

import org.bytedeco.cuda.cudart.CUctx_st;
import org.bytedeco.cuda.cudart.CUstream_st;
import org.bytedeco.cuda.nvjpeg.*;
import org.bytedeco.javacpp.BytePointer;
import org.bytedeco.javacpp.SizeTPointer;
import org.bytedeco.opencv.opencv_core.Mat;

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
      CHECK_CUDA("cuInit", cuInit(0));
      CHECK_CUDA("cuCtxCreate", cuCtxCreate(cudaContext, CU_CTX_SCHED_BLOCKING_SYNC, 0));

      // Initialize stream
      cudaStream = new CUstream_st();
      CHECK_CUDA("cuStreamCreate", cuStreamCreate(cudaStream, NVJPEG_FLAGS_DEFAULT));

      // Initialize handle
      nvjpegHandle = new nvjpegHandle();
      CHECK_NVJPEG("nvjpegCreateSimple", nvjpegCreateSimple(nvjpegHandle));

      // Initialize encoder state
      encoderState = new nvjpegEncoderState();
      CHECK_NVJPEG("nvjpegEncoderStateCreate", nvjpegEncoderStateCreate(nvjpegHandle, encoderState, cudaStream));

      // Initialize encoder parameters
      encoderParameters = new nvjpegEncoderParams();
      CHECK_NVJPEG("nvjpegEncoderParamsCreate", nvjpegEncoderParamsCreate(nvjpegHandle, encoderParameters, cudaStream));

      // Initialize nvjpeg image
      nvjpegImage = new nvjpegImage_t();

      Runtime.getRuntime().addShutdownHook(new Thread(this::destroy, getClass().getSimpleName() + "Shutdown"));
   }

   /**
    * Encodes a YUV I420 image into jpeg.
    * @param sourceImage the YUV I420 image
    * @param outputImagePointer pointer to the jpeg image output
    * @param imageWidth original width (in bytes) of the image
    * @param imageHeight original height (in bytes) of the image
    */
   public void encodeYUV420(BytePointer sourceImage, BytePointer outputImagePointer, int imageWidth, int imageHeight)
   {
      int frameSize = imageWidth * imageHeight;
      long quarterOfFrameSize = ((frameSize % 4 == 0) ? (frameSize / 4) : (frameSize / 4 + 1)); // ensure integer math goes well
      long halfOfImageWidth = ((imageWidth % 2 == 0) ? (imageWidth / 2) : (imageWidth / 2 + 1));

      CHECK_CUDA("cuCtxSetCurrent", cuCtxSetCurrent(cudaContext));

      // Set params to correct sampling factor
      CHECK_NVJPEG("nvjpegEncoderParamsSetSamplingFactors", nvjpegEncoderParamsSetSamplingFactors(encoderParameters, NVJPEG_CSS_420, cudaStream));

      // Set pointer to start of Y plane
      BytePointer planeStartPointer = sourceImage;

      // Get Y plane data
      BytePointer YPlanePointer = new BytePointer(); // create a pointer for the Y plane
      CHECK_CUDA("cudaMalloc", cudaMalloc(YPlanePointer, frameSize)); // allocate Y plane memory
      CHECK_CUDA("cudaMemcpy", cudaMemcpy(YPlanePointer, planeStartPointer, frameSize, cudaMemcpyHostToDevice)); // copy Y plane data to device memory
      nvjpegImage.pitch(0, imageWidth); // set the pitch
      nvjpegImage.channel(0, YPlanePointer); //set the channel
      planeStartPointer.position(planeStartPointer.position() + frameSize); // move plane start pointer to start of U plane

      // Get U plane data
      BytePointer UPlanePointer = new BytePointer();
      CHECK_CUDA("cudaMalloc", cudaMalloc(UPlanePointer, quarterOfFrameSize));
      CHECK_CUDA("cudaMemcpy", cudaMemcpy(UPlanePointer, planeStartPointer, quarterOfFrameSize, cudaMemcpyHostToDevice));
      nvjpegImage.pitch(1, halfOfImageWidth);
      nvjpegImage.channel(1, UPlanePointer);
      planeStartPointer.position(planeStartPointer.position() + quarterOfFrameSize);

      // Get V plane data
      BytePointer VPlanePointer = new BytePointer();
      CHECK_CUDA("cudaMalloc", cudaMalloc(VPlanePointer, quarterOfFrameSize));
      CHECK_CUDA("cudaMemcpy", cudaMemcpy(VPlanePointer, planeStartPointer, quarterOfFrameSize, cudaMemcpyHostToDevice));
      nvjpegImage.pitch(2, halfOfImageWidth);
      nvjpegImage.channel(2, VPlanePointer);

      // Encode image (mem)
      CHECK_NVJPEG("nvjpegEncodeYUV",
                   nvjpegEncodeYUV(nvjpegHandle, encoderState, encoderParameters, nvjpegImage, NVJPEG_CSS_420, imageWidth, imageHeight, cudaStream));

      // Get compressed size
      SizeTPointer jpegSize = new SizeTPointer(1);
      CHECK_NVJPEG("nvjpegEncodeRetrieveBitstream", nvjpegEncodeRetrieveBitstream(nvjpegHandle, encoderState, (BytePointer) null, jpegSize, cudaStream));

      // Retrieve bitstream
      BytePointer jpegBytePointer = new BytePointer(jpegSize.get());
      CHECK_NVJPEG("nvjpegEncodeRetrieveBitstream", nvjpegEncodeRetrieveBitstream(nvjpegHandle, encoderState, jpegBytePointer, jpegSize, cudaStream));

      // Synchronize cuda stream
      CHECK_CUDA("cudaStreamSynchronize", cudaStreamSynchronize(cudaStream));

      //copy data into output pointer
      byte[] bytes = new byte[(int) jpegBytePointer.limit()];
      jpegBytePointer.get(bytes, 0, (int) jpegBytePointer.limit());
      outputImagePointer.put(bytes, 0, (int) jpegBytePointer.limit());
      outputImagePointer.limit(jpegBytePointer.limit());

      // Free GPU memory
      CHECK_CUDA("cudaFree", cudaFree(YPlanePointer));
      CHECK_CUDA("cudaFree", cudaFree(UPlanePointer));
      CHECK_CUDA("cudaFree", cudaFree(VPlanePointer));

      // Close pointers
      YPlanePointer.close();
      UPlanePointer.close();
      VPlanePointer.close();
      jpegSize.close();
   }

   public void encodeBGR(BytePointer sourceImage, BytePointer outputImagePointer, int imageWidth, int imageHeight)
   {
      int numberOfBytesInImage = imageWidth * imageHeight * 3;

      CHECK_CUDA("cuCtxSetCurrent", cuCtxSetCurrent(cudaContext));

      // Set params to correct sampling factor
      CHECK_NVJPEG("nvjpegEncoderParamsSetSamplingFactors", nvjpegEncoderParamsSetSamplingFactors(encoderParameters, NVJPEG_CSS_444, cudaStream));

      // Get B plane data
      BytePointer devicePointer = new BytePointer();
      CHECK_CUDA("cudaMalloc", cudaMalloc(devicePointer, numberOfBytesInImage)); // allocate GPU memory
      CHECK_CUDA("cudaMemcpy", cudaMemcpy(devicePointer, sourceImage, numberOfBytesInImage, cudaMemcpyHostToDevice));
      nvjpegImage.pitch(0, (long) imageWidth * 3);
      nvjpegImage.channel(0, devicePointer);

      // Encode the image
      CHECK_NVJPEG("nvjpegEncodeImage",
                   nvjpegEncodeImage(nvjpegHandle, encoderState, encoderParameters, nvjpegImage, NVJPEG_INPUT_BGRI, imageWidth, imageHeight, cudaStream));

      // Get compressed size
      SizeTPointer jpegSize = new SizeTPointer(1);
      CHECK_NVJPEG("nvjpegEncodeRetrieveBitstream", nvjpegEncodeRetrieveBitstream(nvjpegHandle, encoderState, (BytePointer) null, jpegSize, cudaStream));

      // Retrieve bitstream
      BytePointer jpegBytePointer = new BytePointer(jpegSize.get());
      CHECK_NVJPEG("nvjpegEncodeRetrieveBitstream", nvjpegEncodeRetrieveBitstream(nvjpegHandle, encoderState, jpegBytePointer, jpegSize, cudaStream));

      // Synchronize cuda stream
      CHECK_CUDA("cudaStreamSynchronize", cudaStreamSynchronize(cudaStream));

      //copy data into output pointer
      byte[] bytes = new byte[(int) jpegBytePointer.limit()];
      jpegBytePointer.get(bytes, 0, (int) jpegBytePointer.limit());
      outputImagePointer.put(bytes, 0, (int) jpegBytePointer.limit());
      outputImagePointer.limit(jpegBytePointer.limit());

      // Free GPU memory
      CHECK_CUDA("cudaFree", cudaFree(devicePointer));

      // Close pointers
      jpegSize.close();
   }

   public void destroy()
   {
      CHECK_NVJPEG("nvjpegEncoderParamsDestroy", nvjpegEncoderParamsDestroy(encoderParameters));
      CHECK_NVJPEG("nvjpegEncoderStateDestroy", nvjpegEncoderStateDestroy(encoderState));
      CHECK_NVJPEG("nvjpegDestroy", nvjpegDestroy(nvjpegHandle));
      CHECK_CUDA("cuStreamDestroy", cuStreamDestroy(cudaStream));
      CHECK_CUDA("cuCtxDestroy", cuCtxDestroy(cudaContext));
   }

   /**
    * Helper function for CUDA error checking.
    * Example use:
    *    CHECK_CUDA("cudaFree", cudaFree(devicePointer));
    * @param functionName the name of the function being called as a string
    * @param result the returned error code from the function called. A CUDA function should be called for this parameter
    */
   private static void CHECK_CUDA(String functionName, int result)
   {
      if (result != CUDA_SUCCESS)
      {
         throw new IllegalStateException(String.format("%s returned '%d'", functionName, result));
      }
   }

   /**
    * Helper function for nvjpeg error checking.
    * Example use:
    *    CHECK_NVJPEG("nvjpegCreateSimple", nvjpegCreateSimple(handle));
    * @param functionName the name of the function being called as a string
    * @param result the returned error code from the function called. An nvjpeg function should be called for this parameter
    */
   private static void CHECK_NVJPEG(String functionName, int result)
   {
      if (result != NVJPEG_STATUS_SUCCESS)
      {
         throw new IllegalStateException(String.format("%s returned '%d'", functionName, result));
      }
   }
}
