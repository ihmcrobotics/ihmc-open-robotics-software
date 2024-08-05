package us.ihmc.perception.cuda;

import org.bytedeco.cuda.cudart.CUstream_st;
import org.bytedeco.cuda.nvjpeg.nvjpegEncoderParams;
import org.bytedeco.cuda.nvjpeg.nvjpegEncoderState;
import org.bytedeco.cuda.nvjpeg.nvjpegHandle;
import org.bytedeco.cuda.nvjpeg.nvjpegImage_t;
import org.bytedeco.cuda.nvjpeg.nvjpegJpegState;
import org.bytedeco.javacpp.BytePointer;
import org.bytedeco.javacpp.IntPointer;
import org.bytedeco.javacpp.SizeTPointer;
import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.global.opencv_imgproc;
import org.bytedeco.opencv.opencv_core.GpuMat;
import org.bytedeco.opencv.opencv_core.Mat;

import static org.bytedeco.cuda.global.cudart.*;
import static org.bytedeco.cuda.global.nvjpeg.*;
import static us.ihmc.perception.cuda.CUDATools.checkCUDAError;
import static us.ihmc.perception.cuda.CUDATools.checkNVJPEGError;

/**
 * Used for encoding images using CUDA.
 * Ensure that the computer has CUDA available before using this class. Most all Nvidia graphics cards should have CUDA available.
 *
 * This class has been adapted from the SampleJpegEncoder of bytedeco:
 * https://github.com/bytedeco/javacpp-presets/blob/master/cuda/samples/SampleJpegEncoder.java
 */
public class CUDAImageEncoder
{
   private final CUstream_st cudaStream;
   private final nvjpegHandle nvjpegHandle;
   private final nvjpegJpegState nvjpegState;
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

      // Initialize nvjpeg state (used for decoding)
      nvjpegState = new nvjpegJpegState();
      checkNVJPEGError(nvjpegJpegStateCreate(nvjpegHandle, nvjpegState));

      // Initialize encoder state
      encoderState = new nvjpegEncoderState();
      checkNVJPEGError(nvjpegEncoderStateCreate(nvjpegHandle, encoderState, cudaStream));

      // Initialize encoder parameters
      encoderParameters = new nvjpegEncoderParams();
      checkNVJPEGError(nvjpegEncoderParamsCreate(nvjpegHandle, encoderParameters, cudaStream));
   }

   /**
    * Encodes a YUV I420 image into jpeg.
    *
    * @param sourceImage        the YUV I420 image
    * @param outputImagePointer pointer to the jpeg image output
    * @param imageWidth         width (in bytes) of the source image
    * @param imageHeight        height (in bytes) of the source image
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

   public void encodeBGR(Mat imageToEncode, BytePointer encodedImage)
   {
      encodeInterleaved(imageToEncode.data(),
                        imageToEncode.cols(),
                        imageToEncode.rows(),
                        imageToEncode.elemSize(),
                        imageToEncode.step(),
                        NVJPEG_INPUT_BGRI,
                        encodedImage);
   }

   public void encodeBGR(GpuMat imageToEncode, BytePointer encodedImage)
   {
      encodeInterleaved(imageToEncode.data(),
                        imageToEncode.cols(),
                        imageToEncode.rows(),
                        imageToEncode.elemSize(),
                        imageToEncode.step(),
                        NVJPEG_INPUT_BGRI,
                        encodedImage);
   }

   public void encodeRGB(Mat imageToEncode, BytePointer encodedImage)
   {
      encodeInterleaved(imageToEncode.data(),
                        imageToEncode.cols(),
                        imageToEncode.rows(),
                        imageToEncode.elemSize(),
                        imageToEncode.step(),
                        NVJPEG_INPUT_RGBI,
                        encodedImage);
   }

   public void encodeRGB(GpuMat imageToEncode, BytePointer encodedImage)
   {
      encodeInterleaved(imageToEncode.data(),
                        imageToEncode.cols(),
                        imageToEncode.rows(),
                        imageToEncode.elemSize(),
                        imageToEncode.step(),
                        NVJPEG_INPUT_RGBI,
                        encodedImage);
   }

   public void encodeGray(Mat imageToEncode, BytePointer encodedImage)
   {
      GpuMat gpuImageToEncode = new GpuMat(imageToEncode.size(), imageToEncode.type());
      gpuImageToEncode.upload(imageToEncode);
      encodeGray(gpuImageToEncode, encodedImage);
      gpuImageToEncode.close();
   }

   public void encodeGray(GpuMat imageToEncode, BytePointer encodedImage)
   {
      GpuMat bgrImage = new GpuMat();
      opencv_imgproc.cvtColor(imageToEncode, bgrImage, opencv_imgproc.CV_GRAY2BGR);
      encodeInterleaved(bgrImage.data(), bgrImage.cols(), bgrImage.rows(), bgrImage.elemSize(), bgrImage.step(), NVJPEG_INPUT_BGRI, encodedImage);
      bgrImage.close();
   }

   /**
    * Encodes an interleaved image into jpeg
    *
    * @param imageToEncode          the BGR image to encode
    * @param imageWidth             width of the source image
    * @param imageHeight            height of the source image
    * @param elementSize            element size (in bytes) of the source image
    * @param imagePitch             pitch (aka step in OpenCV land) of the source image
    * @param interleavedInputFormat either NVJPEG_INPUT_BGRI or NVJPEG_INPUT_RGBI
    * @param encodedImage           output pointer for encoded data
    */
   public void encodeInterleaved(BytePointer imageToEncode,
                                 int imageWidth,
                                 int imageHeight,
                                 long elementSize,
                                 long imagePitch,
                                 int interleavedInputFormat,
                                 BytePointer encodedImage)
   {
      long rowSize = elementSize * imageWidth;
      long totalSize = elementSize * imageWidth * imageHeight;

      // Set sampling factor
      checkNVJPEGError(nvjpegEncoderParamsSetSamplingFactors(encoderParameters, NVJPEG_CSS_444, cudaStream));

      // Upload image data to device
      BytePointer deviceImagePointer = new BytePointer();
      checkCUDAError(cudaMallocAsync(deviceImagePointer, totalSize, cudaStream));
      checkCUDAError(cudaMemcpy2DAsync(deviceImagePointer, rowSize, imageToEncode, imagePitch, rowSize, imageHeight, cudaMemcpyDefault, cudaStream));

      // Create nvjpeg image
      nvjpegImage_t nvjpegImage = new nvjpegImage_t();
      nvjpegImage.pitch(0, rowSize);
      nvjpegImage.channel(0, deviceImagePointer);

      // Encode the image
      checkNVJPEGError(nvjpegEncodeImage(nvjpegHandle,
                                         encoderState,
                                         encoderParameters,
                                         nvjpegImage,
                                         interleavedInputFormat,
                                         imageWidth,
                                         imageHeight,
                                         cudaStream));

      // Get compressed image size
      SizeTPointer jpegSize = new SizeTPointer(1);
      checkNVJPEGError(nvjpegEncodeRetrieveBitstream(nvjpegHandle, encoderState, (BytePointer) null, jpegSize, cudaStream));

      // Retrieve bitstream
      encodedImage.limit(jpegSize.get());
      checkNVJPEGError(nvjpegEncodeRetrieveBitstream(nvjpegHandle, encoderState, encodedImage, jpegSize, cudaStream));

      // Free GPU memory
      checkCUDAError(cudaFreeAsync(deviceImagePointer, cudaStream));

      // close everything
      deviceImagePointer.close();
      jpegSize.close();
      nvjpegImage.close();
   }

   public void decodeToBGR(BytePointer encodedImage, long encodedImageSize, Mat decodedImage)
   {
      // Get decoded image info
      NVJPEGImageInfo imageInfo = getImageData(encodedImage, encodedImageSize);

      // Create appropriately sized Mat & allocate host memory for the Mat
      long decodedImageSize = 3L * imageInfo.widths[0] * imageInfo.heights[0];
      BytePointer decodedData = new BytePointer();
      checkCUDAError(cudaMallocHost(decodedData, decodedImageSize));

      // Decode the image, packing result into Mat data
      decodeBGR(encodedImage, encodedImageSize, imageInfo, decodedData);

      // Pack the result into the output Mat
      Mat decodingResult = new Mat(imageInfo.heights[0], imageInfo.widths[0], opencv_core.CV_8UC3, decodedData);
      decodingResult.copyTo(decodedImage);

      // Free all memory
      checkCUDAError(cudaFreeHost(decodedData));
      decodingResult.close();
   }

   public NVJPEGImageInfo getImageData(BytePointer encodedImage, long encodedImageSize)
   {
      try (IntPointer numberOfComponents = new IntPointer();
           IntPointer subSamplingsPointer = new IntPointer();
           IntPointer widthsPointer = new IntPointer();
           IntPointer heightsPointer = new IntPointer())
      {
         // Allocate host memory to receive data
         checkCUDAError(cudaMallocHost(numberOfComponents, 1));
         checkCUDAError(cudaMallocHost(subSamplingsPointer, NVJPEG_MAX_COMPONENT));
         checkCUDAError(cudaMallocHost(widthsPointer, NVJPEG_MAX_COMPONENT));
         checkCUDAError(cudaMallocHost(heightsPointer, NVJPEG_MAX_COMPONENT));

         // Extract the data
         checkNVJPEGError(nvjpegGetImageInfo(nvjpegHandle,
                                             encodedImage,
                                             encodedImageSize,
                                             numberOfComponents,
                                             subSamplingsPointer,
                                             widthsPointer,
                                             heightsPointer));

         int components = numberOfComponents.get();
         int[] subSamplings = new int[components];
         int[] widths = new int[components];
         int[] heights = new int[components];

         for (int i = 0; i < components; ++i)
         {
            subSamplings[i] = subSamplingsPointer.get(i);
            widths[i] = widthsPointer.get(i);
            heights[i] = heightsPointer.get(i);
         }

         NVJPEGImageInfo imageInfo = new NVJPEGImageInfo(numberOfComponents.get(), subSamplings, widths, heights);

         // Deallocate host memory
         checkCUDAError(cudaFreeHost(numberOfComponents));
         checkCUDAError(cudaFreeHost(subSamplingsPointer));
         checkCUDAError(cudaFreeHost(widthsPointer));
         checkCUDAError(cudaFreeHost(heightsPointer));

         return imageInfo;
      }
   }

   public void decodeBGR(BytePointer encodedImage, long encodedImageSize, NVJPEGImageInfo decodedImageInfo, BytePointer decodedImage)
   {
      // Allocate device memory for decoded image
      BytePointer decodedDeviceImage = new BytePointer();
      long decodedImageSize = 3L * decodedImageInfo.widths[0] * decodedImageInfo.heights[0];
      checkCUDAError(cudaMallocAsync(decodedDeviceImage, decodedImageSize, cudaStream));

      // Create NVJPEG image
      nvjpegImage_t nvjpegDecodedImage = new nvjpegImage_t();
      nvjpegDecodedImage.pitch(0, 3L * decodedImageInfo.widths[0]);
      nvjpegDecodedImage.channel(0, decodedDeviceImage);

      // Decode the image
      checkNVJPEGError(nvjpegDecode(nvjpegHandle, nvjpegState, encodedImage, encodedImageSize, NVJPEG_OUTPUT_BGRI, nvjpegDecodedImage, cudaStream));

      // Copy data to output pointer
      checkCUDAError(cudaMemcpyAsync(decodedImage, decodedDeviceImage, decodedImageSize, cudaMemcpyDefault, cudaStream));
      checkCUDAError(cudaStreamSynchronize(cudaStream));

      checkCUDAError(cudaFreeAsync(decodedDeviceImage, cudaStream));
      decodedDeviceImage.close();
   }

   private record NVJPEGImageInfo(int numberOfComponents, int[] subSamplingTypes, int[] widths, int[] heights)
   {
   }

   public void destroy()
   {
      checkNVJPEGError(nvjpegEncoderParamsDestroy(encoderParameters));
      checkNVJPEGError(nvjpegEncoderStateDestroy(encoderState));
      checkNVJPEGError(nvjpegJpegStateDestroy(nvjpegState));
      checkNVJPEGError(nvjpegDestroy(nvjpegHandle));
      CUDAStreamManager.releaseStream(cudaStream);
   }
}
