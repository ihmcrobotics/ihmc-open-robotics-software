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
import static us.ihmc.perception.cuda.CUDATools.checkNVJPEGError;

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

   /**
    * Initialize necessary CUDA components
    */
   public CUDAImageEncoder()
   {
      // Initialize stream
      cudaStream = CUDAStreamManager.getStream();

      // Initialize handle
      nvjpegHandle = new nvjpegHandle();
      checkNVJPEGError(nvjpegCreateSimple(nvjpegHandle));

      // Initialize encoder state
      encoderState = new nvjpegEncoderState();
      checkNVJPEGError(nvjpegEncoderStateCreate(nvjpegHandle, encoderState, cudaStream));

      // Initialize encoder parameters
      encoderParameters = new nvjpegEncoderParams();
      checkNVJPEGError(nvjpegEncoderParamsCreate(nvjpegHandle, encoderParameters, cudaStream));
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
      checkNVJPEGError(nvjpegEncoderParamsSetSamplingFactors(encoderParameters, NVJPEG_CSS_420, cudaStream));

      // Get Y plane data
      BytePointer yPlanePointer = new BytePointer(); // create a pointer for the Y plane
      checkCUDAError(cudaMallocAsync(yPlanePointer, frameSize, cudaStream)); // allocate Y plane memory
      checkCUDAError(cudaMemcpyAsync(yPlanePointer, sourceImage, frameSize, cudaMemcpyHostToDevice, cudaStream)); // copy Y plane data to device memory

      nvjpegImage_t nvjpegImage = new nvjpegImage_t();
      nvjpegImage.pitch(0, imageWidth); // set the pitch
      nvjpegImage.channel(0, yPlanePointer); //set the channel
      sourceImage.position(sourceImage.position() + frameSize); // move pointer to start of U plane

      // Get U plane data
      BytePointer uPlanePointer = new BytePointer();
      checkCUDAError(cudaMallocAsync(uPlanePointer, quarterOfFrameSize, cudaStream));
      checkCUDAError(cudaMemcpyAsync(uPlanePointer, sourceImage, quarterOfFrameSize, cudaMemcpyHostToDevice, cudaStream));
      nvjpegImage.pitch(1, halfOfImageWidth);
      nvjpegImage.channel(1, uPlanePointer);
      sourceImage.position(sourceImage.position() + quarterOfFrameSize);

      // Get V plane data
      BytePointer vPlanePointer = new BytePointer();
      checkCUDAError(cudaMallocAsync(vPlanePointer, quarterOfFrameSize, cudaStream));
      checkCUDAError(cudaMemcpyAsync(vPlanePointer, sourceImage, quarterOfFrameSize, cudaMemcpyHostToDevice, cudaStream));
      nvjpegImage.pitch(2, halfOfImageWidth);
      nvjpegImage.channel(2, vPlanePointer);

      // Encode image (mem)
      checkNVJPEGError(nvjpegEncodeYUV(nvjpegHandle, encoderState, encoderParameters, nvjpegImage, NVJPEG_CSS_420, imageWidth, imageHeight, cudaStream));

      // Get compressed size
      SizeTPointer jpegSize = new SizeTPointer(1);
      checkNVJPEGError(nvjpegEncodeRetrieveBitstream(nvjpegHandle, encoderState, (BytePointer) null, jpegSize, cudaStream));

      // Retrieve bitstream
      outputImagePointer.limit(jpegSize.get());
      checkNVJPEGError(nvjpegEncodeRetrieveBitstream(nvjpegHandle, encoderState, outputImagePointer, jpegSize, cudaStream));

      // Free GPU memory
      checkCUDAError(cudaFreeAsync(yPlanePointer, cudaStream));
      checkCUDAError(cudaFreeAsync(uPlanePointer, cudaStream));
      checkCUDAError(cudaFreeAsync(vPlanePointer, cudaStream));

      // Close pointers
      yPlanePointer.close();
      uPlanePointer.close();
      vPlanePointer.close();
      jpegSize.close();
      nvjpegImage.close();
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
      checkNVJPEGError(nvjpegEncoderParamsSetSamplingFactors(encoderParameters, NVJPEG_CSS_444, cudaStream));

      // Upload image data
      BytePointer devicePointer = new BytePointer();
      checkCUDAError(cudaMallocAsync(devicePointer, frameSize, cudaStream)); // allocate GPU memory
      checkCUDAError(cudaMemcpy2DAsync(devicePointer, rowSize, sourceImage, sourceImagePitch, rowSize, imageHeight, cudaMemcpyDefault, cudaStream));

      nvjpegImage_t nvjpegImage = new nvjpegImage_t();
      nvjpegImage.pitch(0, rowSize);
      nvjpegImage.channel(0, devicePointer);

      // Encode the image
      checkNVJPEGError(nvjpegEncodeImage(nvjpegHandle, encoderState, encoderParameters, nvjpegImage, NVJPEG_INPUT_BGRI, imageWidth, imageHeight, cudaStream));

      // Get compressed size
      SizeTPointer jpegSize = new SizeTPointer(1);
      checkNVJPEGError(nvjpegEncodeRetrieveBitstream(nvjpegHandle, encoderState, (BytePointer) null, jpegSize, cudaStream));

      // Retrieve bitstream
      outputImagePointer.limit(jpegSize.get());
      checkNVJPEGError(nvjpegEncodeRetrieveBitstream(nvjpegHandle, encoderState, outputImagePointer, jpegSize, cudaStream));

      // Free GPU memory
      checkCUDAError(cudaFreeAsync(devicePointer, cudaStream));

      // Close pointers
      jpegSize.close();
      nvjpegImage.close();
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
      checkNVJPEGError(nvjpegEncoderParamsSetSamplingFactors(encoderParameters, NVJPEG_CSS_444, cudaStream));

      // Get B plane data
      BytePointer devicePointer = new BytePointer();
      checkCUDAError(cudaMallocAsync(devicePointer, frameSize, cudaStream)); // allocate GPU memory
      checkCUDAError(cudaMemcpy2DAsync(devicePointer, rowSize, sourceImage, sourceImagePitch, rowSize, imageHeight, cudaMemcpyDefault, cudaStream));

      nvjpegImage_t nvjpegImage = new nvjpegImage_t();
      nvjpegImage.pitch(0, rowSize);
      nvjpegImage.channel(0, devicePointer);

      // Encode the image
      checkNVJPEGError(nvjpegEncodeImage(nvjpegHandle, encoderState, encoderParameters, nvjpegImage, NVJPEG_INPUT_RGBI, imageWidth, imageHeight, cudaStream));

      // Get compressed size
      SizeTPointer jpegSize = new SizeTPointer(1);
      checkNVJPEGError(nvjpegEncodeRetrieveBitstream(nvjpegHandle, encoderState, (BytePointer) null, jpegSize, cudaStream));

      // Retrieve bitstream
      outputImagePointer.limit(jpegSize.get());
      checkNVJPEGError(nvjpegEncodeRetrieveBitstream(nvjpegHandle, encoderState, outputImagePointer, jpegSize, cudaStream));

      // Free GPU memory
      checkCUDAError(cudaFreeAsync(devicePointer, cudaStream));

      // Close pointers
      jpegSize.close();
      nvjpegImage.close();
   }

   public void destroy()
   {
      checkNVJPEGError(nvjpegEncoderParamsDestroy(encoderParameters));
      checkNVJPEGError(nvjpegEncoderStateDestroy(encoderState));
      checkNVJPEGError(nvjpegDestroy(nvjpegHandle));
   }
}
