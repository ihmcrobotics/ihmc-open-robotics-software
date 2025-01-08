package us.ihmc.perception.cuda;

import org.bytedeco.cuda.cudart.CUstream_st;
import org.bytedeco.cuda.nvjpeg.nvjpegEncoderParams;
import org.bytedeco.cuda.nvjpeg.nvjpegEncoderState;
import org.bytedeco.cuda.nvjpeg.nvjpegHandle;
import org.bytedeco.cuda.nvjpeg.nvjpegImage_t;
import org.bytedeco.cuda.nvjpeg.nvjpegJpegState;
import org.bytedeco.javacpp.BytePointer;
import org.bytedeco.javacpp.IntPointer;
import org.bytedeco.javacpp.Pointer;
import org.bytedeco.javacpp.SizeTPointer;
import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.global.opencv_cudaarithm;
import org.bytedeco.opencv.global.opencv_cudaimgproc;
import org.bytedeco.opencv.global.opencv_imgproc;
import org.bytedeco.opencv.opencv_core.GpuMat;
import org.bytedeco.opencv.opencv_core.GpuMatVector;
import org.bytedeco.opencv.opencv_core.Mat;
import org.bytedeco.opencv.opencv_core.MatVector;

import java.util.ArrayList;
import java.util.List;

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
public class CUDAJPEGProcessor
{
   private final CUstream_st cudaStream;
   private final nvjpegHandle nvjpegHandle;
   private final nvjpegJpegState nvjpegState;
   private final nvjpegEncoderState encoderState;
   private final nvjpegEncoderParams encoderParameters;

   public CUDAJPEGProcessor()
   {
      this(75);
   }

   /**
    * Initialize necessary CUDA components
    * @param quality value between 1 and 100 representing the quality of the jpeg image. 1 will result in the lowest quality, but highest compression.
    */
   public CUDAJPEGProcessor(int quality)
   {
      if (!CUDATools.hasNVJPEG())
         throw new UnsatisfiedLinkError("NVJPEG was not found. To install NVJPEG, see instructions in ihmc-perception/README.md");

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
      checkNVJPEGError(nvjpegEncoderParamsSetQuality(encoderParameters, quality, cudaStream));
   }

   /**
    * Encodes a YUV I420 image into jpeg.
    *
    * @param sourceImage        the YUV I420 image
    * @param encodedImage pointer to the jpeg image output
    */
   public void encodeYUV420(Pointer sourceImage, BytePointer encodedImage)
   {
      GpuMat gpuSourceImage = new GpuMat();
      if (sourceImage instanceof Mat cpuMat)
         gpuSourceImage.upload(cpuMat);
      else if (sourceImage instanceof GpuMat gpuImage)
         gpuImage.copyTo(gpuSourceImage);

      int totalHeight = gpuSourceImage.rows();
      int lumaHeight = (2 * totalHeight) / 3;

      int width = gpuSourceImage.cols();
      int chromaWidth = width / 2;

      GpuMatVector planes = new GpuMatVector();
      planes.push_back(gpuSourceImage.rowRange(0, lumaHeight));                                          // Y
      planes.push_back(gpuSourceImage.rowRange(lumaHeight, totalHeight).colRange(0, chromaWidth));       // U
      planes.push_back(gpuSourceImage.rowRange(lumaHeight, totalHeight).colRange(chromaWidth, width));   // V

      encodeYUV(planes, NVJPEG_CSS_420, encodedImage);

      planes.close();
      gpuSourceImage.close();
   }

   public void encodeYUV444(Pointer sourceImage, BytePointer encodedImage)
   {
      GpuMat gpuSourceImage = new GpuMat();
      if (sourceImage instanceof Mat cpuMat)
         gpuSourceImage.upload(cpuMat);
      else if (sourceImage instanceof GpuMat gpuImage)
         gpuImage.copyTo(gpuSourceImage);

      GpuMatVector planes = new GpuMatVector();
      opencv_cudaarithm.split(gpuSourceImage, planes);

      encodeYUV(planes, NVJPEG_CSS_444, encodedImage);

      planes.close();
      gpuSourceImage.close();
   }

   public void encodeYUV(GpuMatVector imageToEncode, int chromaSubSampling, BytePointer encodedImage)
   {
      // Set sampling factor
      checkNVJPEGError(nvjpegEncoderParamsSetSamplingFactors(encoderParameters, chromaSubSampling, cudaStream));

      // Create nvjpeg image
      nvjpegImage_t nvjpegImage = new nvjpegImage_t();
      for (int i = 0; i < imageToEncode.size() && i < 4; ++i)
      {
         GpuMat plane = imageToEncode.get(i);
         nvjpegImage.pitch(i, plane.step());
         nvjpegImage.channel(i, plane.data());
      }

      GpuMat lumaPlane = imageToEncode.get(0);
      checkNVJPEGError(nvjpegEncodeYUV(nvjpegHandle,
                                       encoderState,
                                       encoderParameters,
                                       nvjpegImage,
                                       chromaSubSampling,
                                       lumaPlane.cols(),
                                       lumaPlane.rows(),
                                       cudaStream));

      // Get compressed image size
      SizeTPointer jpegSize = new SizeTPointer(1);
      checkNVJPEGError(nvjpegEncodeRetrieveBitstream(nvjpegHandle, encoderState, (BytePointer) null, jpegSize, cudaStream));

      // Retrieve bitstream
      encodedImage.limit(jpegSize.get());
      checkNVJPEGError(nvjpegEncodeRetrieveBitstream(nvjpegHandle, encoderState, encodedImage, jpegSize, cudaStream));

      checkCUDAError(cudaStreamSynchronize(cudaStream));

      // Close everything
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
      opencv_cudaimgproc.cvtColor(imageToEncode, bgrImage, opencv_imgproc.CV_GRAY2BGR);
      encodeBGR(bgrImage, encodedImage);
      bgrImage.close();
   }

   /**
    * Encodes an interleaved image into jpeg
    *
    * @param imageToEncode          the BGR image to encode
    * @param imageWidth             width of the source image
    * @param imageHeight            height of the source image
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

   /**
    * Decodes a jpeg encoded image to BGR format.
    * @param encodedImage INPUT: An encoded multi-channel image.
    * @param encodedImageSize INPUT: Number of bytes of the encoded image.
    * @param decodedImage OUTPUT: The decoded image.
    */
   public void decodeToBGR(BytePointer encodedImage, long encodedImageSize, Mat decodedImage)
   {
      // Get decoded image info
      NVJPEGImageInfo imageInfo = getImageInfo(encodedImage, encodedImageSize);

      // Allocate host memory for the decoded image
      long decodedImageSize = getOutputChannelSizes(NVJPEG_OUTPUT_BGRI, imageInfo)[0];
      List<BytePointer> decodedData = new ArrayList<>();
      decodedData.add(new BytePointer());
      checkCUDAError(cudaMallocHost(decodedData.get(0), decodedImageSize));

      // Decode the image, packing result into the allocated buffer.
      decodeImage(encodedImage, encodedImageSize, imageInfo, NVJPEG_OUTPUT_BGRI, decodedData);

      // Pack the result into the output Mat
      Mat decodingResult = new Mat(imageInfo.height(0), imageInfo.width(0), opencv_core.CV_8UC3, decodedData.get(0));
      decodingResult.copyTo(decodedImage);

      // Free all memory
      checkCUDAError(cudaFreeHost(decodedData.get(0)));
      decodedData.get(0).close();
      decodingResult.close();
   }

   /**
    * Decodes a jpeg encoded image to BGR format.
    * @param encodedImage INPUT: An encoded multi-channel image.
    * @param encodedImageSize INPUT: Number of bytes of the encoded image.
    * @param decodedImage OUTPUT: The decoded image.
    */
   public void decodeToBGR(BytePointer encodedImage, long encodedImageSize, GpuMat decodedImage)
   {
      // Get decoded image info
      NVJPEGImageInfo imageInfo = getImageInfo(encodedImage, encodedImageSize);

      // Allocate device memory for the decoded image
      long decodedImageSize = getOutputChannelSizes(NVJPEG_OUTPUT_BGRI, imageInfo)[0];
      List<BytePointer> decodedData = new ArrayList<>();
      decodedData.add(new BytePointer());
      checkCUDAError(cudaMallocAsync(decodedData.get(0), decodedImageSize, cudaStream));

      decodeImage(encodedImage, encodedImageSize, imageInfo, NVJPEG_OUTPUT_BGRI, decodedData);

      GpuMat decodingResult = new GpuMat(imageInfo.height(0), imageInfo.width(0), opencv_core.CV_8UC3, decodedData.get(0));
      decodingResult.copyTo(decodedImage);

      // Free all memory
      checkCUDAError(cudaFreeAsync(decodedData.get(0), cudaStream));
      decodedData.get(0).close();
      decodingResult.close();
   }

   /**
    * Decodes a jpeg encoded image to YUV I420 format.
    * @param encodedImage INPUT: An encoded multi-channel image.
    * @param encodedImageSize INPUT: Number of bytes of the encoded image.
    * @param decodedImage OUTPUT: The decoded image.
    * @apiNote Despite the name, this method does not work when the passed in encoded image is in an OpenCV YUV format.
    * To decode an OpenCV YUV_I420 image, use {@link CUDAJPEGProcessor#decodeUnchanged(BytePointer, long, Mat)}.
    * While this method returns an image when the input is an OpenCV YUV image, the colors are incorrect.
    * TODO: Find out why the colors are incorrect when providing an OpenCV YUV image.
    */
   public void decodeToYUV(BytePointer encodedImage, long encodedImageSize, Mat decodedImage)
   {
      // Get decoded image info
      NVJPEGImageInfo imageInfo = getImageInfo(encodedImage, encodedImageSize);

      // Allocate host memory for the decoded image channels (Y, U, and V)
      int numberOfDecodedChannels = getNumberOfOutputChannels(NVJPEG_OUTPUT_YUV, imageInfo);
      long[] decodedChannelSizes = getOutputChannelSizes(NVJPEG_OUTPUT_YUV, imageInfo);
      List<BytePointer> decodedChannels = new ArrayList<>(numberOfDecodedChannels);
      for (int i = 0; i < numberOfDecodedChannels; ++i)
      {
         decodedChannels.add(new BytePointer());
         checkCUDAError(cudaMallocHost(decodedChannels.get(i), decodedChannelSizes[i]));
      }

      // Decode the image, packing result into the allocated buffers.
      decodeImage(encodedImage, encodedImageSize, imageInfo, NVJPEG_OUTPUT_YUV, decodedChannels);

      // Create Mats representing each Y, U, and V plane
      try (Mat yPlane = new Mat(imageInfo.height(0), imageInfo.width(0), opencv_core.CV_8UC1, decodedChannels.get(0));
           Mat uPlane = new Mat(imageInfo.height(1), imageInfo.width(1), opencv_core.CV_8UC1, decodedChannels.get(1));
           Mat vPlane = new Mat(imageInfo.height(2), imageInfo.width(2), opencv_core.CV_8UC1, decodedChannels.get(2));
           Mat yuvI420CombinedImage = new Mat(imageInfo.height(0) + imageInfo.height(1), imageInfo.width(0), opencv_core.CV_8UC1))
      {
         // Pack the YUV planes into their respective locations
         yPlane.copyTo(yuvI420CombinedImage.rowRange(0, imageInfo.height(0)));
         uPlane.copyTo(yuvI420CombinedImage.rowRange(imageInfo.height(0), imageInfo.height(0) + imageInfo.height(1)).colRange(0, imageInfo.width(1)));
         vPlane.copyTo(yuvI420CombinedImage.rowRange(imageInfo.height(0), imageInfo.height(0) + imageInfo.height(2))
                                           .colRange(imageInfo.width(1), imageInfo.width(1) + imageInfo.width(2)));
         // Copy the YUV image into the output image
         yuvI420CombinedImage.copyTo(decodedImage);
      }

      // Free all memory
      for (int i = 0; i < numberOfDecodedChannels; ++i)
      {
         checkCUDAError(cudaFreeHost(decodedChannels.get(i)));
         decodedChannels.get(i).close();
      }
   }

   public void decodeToGray(BytePointer encodedImage, long encodedImageSize, Mat decodedImage)
   {
      NVJPEGImageInfo imageInfo = getImageInfo(encodedImage, encodedImageSize);

      // Allocate host memory for the decoded image
      long decodedImageSize = getOutputChannelSizes(NVJPEG_OUTPUT_Y, imageInfo)[0];
      List<BytePointer> decodedData = new ArrayList<>();
      decodedData.add(new BytePointer());
      checkCUDAError(cudaMallocHost(decodedData.get(0), decodedImageSize));

      // Decode only Y plane (luminosity)
      decodeImage(encodedImage, encodedImageSize, imageInfo, NVJPEG_OUTPUT_Y, decodedData);

      // Copy result into output image
      Mat decodingResult = new Mat(imageInfo.height(0), imageInfo.width(0), opencv_core.CV_8UC1, decodedData.get(0));
      decodingResult.copyTo(decodedImage);

      // Free all memory
      checkCUDAError(cudaFreeHost(decodedData.get(0)));
      decodedData.get(0).close();
      decodingResult.close();
   }

   public void decodeToGray(BytePointer encodedImage, long encodedImageSize, GpuMat decodedImage)
   {
      NVJPEGImageInfo imageInfo = getImageInfo(encodedImage, encodedImageSize);

      // Allocate host memory for the decoded image
      long decodedImageSize = getOutputChannelSizes(NVJPEG_OUTPUT_Y, imageInfo)[0];
      List<BytePointer> decodedData = new ArrayList<>();
      decodedData.add(new BytePointer());
      checkCUDAError(cudaMallocAsync(decodedData.get(0), decodedImageSize, cudaStream));

      // Decode only Y plane (luminosity)
      decodeImage(encodedImage, encodedImageSize, imageInfo, NVJPEG_OUTPUT_Y, decodedData);

      // Copy result into output image
      GpuMat decodingResult = new GpuMat(imageInfo.height(0), imageInfo.width(0), opencv_core.CV_8UC1, decodedData.get(0));
      decodingResult.copyTo(decodedImage);

      // Free all memory
      checkCUDAError(cudaFreeAsync(decodedData.get(0), cudaStream));
      decodedData.get(0).close();
      decodingResult.close();
   }

   /**
    * Decodes a jpeg encoded image, leaving the jpeg's format as is.
    * If the jpeg has multiple channels, the output format is likely to be YUV.
    * If the jpeg has a single channel, the output format will be gray scale.
    * @param encodedImage INPUT: An encoded multi-channel image.
    * @param encodedImageSize INPUT: Number of bytes of the encoded image.
    * @param decodedImage OUTPUT: The decoded image.
    */
   public void decodeUnchanged(BytePointer encodedImage, long encodedImageSize, Mat decodedImage)
   {
      // Get decoded image info
      NVJPEGImageInfo imageInfo = getImageInfo(encodedImage, encodedImageSize);

      // Allocate host memory for the decoded image channels
      int numberOfDecodedChannels = imageInfo.numberOfComponents();
      long[] decodedChannelSizes = getOutputChannelSizes(NVJPEG_OUTPUT_UNCHANGED, imageInfo);
      List<BytePointer> decodedChannels = new ArrayList<>(numberOfDecodedChannels);
      for (int i = 0; i < numberOfDecodedChannels; ++i)
      {
         decodedChannels.add(new BytePointer());
         checkCUDAError(cudaMallocHost(decodedChannels.get(i), decodedChannelSizes[i]));
      }

      // Decode the image, packing result into the allocated buffers.
      decodeImage(encodedImage, encodedImageSize, imageInfo, NVJPEG_OUTPUT_UNCHANGED, decodedChannels);

      // Put all channels into a MatVector
      MatVector decodedChannelMats = new MatVector();
      for (int i = 0; i < numberOfDecodedChannels; ++i)
      {
         Mat channelMat = new Mat(imageInfo.maxHeight(), imageInfo.maxWidth(), opencv_core.CV_8UC1, decodedChannels.get(i));
         decodedChannelMats.push_back(channelMat);
      }

      // Combine the channels into 1 Mat, pack into output image
      opencv_core.merge(decodedChannelMats, decodedImage);

      // Free all memory
      for (int i = 0; i < numberOfDecodedChannels; ++i)
      {
         checkCUDAError(cudaFreeHost(decodedChannels.get(i)));
         decodedChannels.get(i).close();
      }
   }

   /**
    * Decodes a jpeg encoded image, leaving the jpeg's format as is.
    * If the jpeg has multiple channels, the output format is likely to be YUV.
    * If the jpeg has a single channel, the output format will be gray scale.
    * @param encodedImage INPUT: An encoded multi-channel image.
    * @param encodedImageSize INPUT: Number of bytes of the encoded image.
    * @param decodedImage OUTPUT: The decoded image.
    */
   public void decodeUnchanged(BytePointer encodedImage, long encodedImageSize, GpuMat decodedImage)
   {
      // Get decoded image info
      NVJPEGImageInfo imageInfo = getImageInfo(encodedImage, encodedImageSize);

      // Allocate host memory for the decoded image channels
      int numberOfDecodedChannels = imageInfo.numberOfComponents();
      long[] decodedChannelSizes = getOutputChannelSizes(NVJPEG_OUTPUT_UNCHANGED, imageInfo);
      List<BytePointer> decodedChannels = new ArrayList<>(numberOfDecodedChannels);
      for (int i = 0; i < numberOfDecodedChannels; ++i)
      {
         decodedChannels.add(new BytePointer());
         checkCUDAError(cudaMallocAsync(decodedChannels.get(i), decodedChannelSizes[i], cudaStream));
      }

      // Decode the image, packing result into the allocated buffers.
      decodeImage(encodedImage, encodedImageSize, imageInfo, NVJPEG_OUTPUT_UNCHANGED, decodedChannels);

      // Put all channels into a MatVector
      GpuMatVector decodedChannelMats = new GpuMatVector();
      for (int i = 0; i < numberOfDecodedChannels; ++i)
      {
         GpuMat channelMat = new GpuMat(imageInfo.maxHeight(), imageInfo.maxWidth(), opencv_core.CV_8UC1, decodedChannels.get(i));
         decodedChannelMats.push_back(channelMat);
      }

      // Combine the channels into 1 Mat, pack into output image
      opencv_cudaarithm.merge(decodedChannelMats, decodedImage);

      // Free all memory
      for (int i = 0; i < numberOfDecodedChannels; ++i)
      {
         checkCUDAError(cudaFreeAsync(decodedChannels.get(i), cudaStream));
         decodedChannels.get(i).close();
      }
   }

   /**
    * Decodes a jpeg encoded image into the specified output type.
    * @param encodedImage INPUT: An encoded multi-channel image.
    * @param encodedImageSize INPUT: Number of bytes of the encoded image.
    * @param decodedImageInfo INPUT: {@link NVJPEGImageInfo} about the passed in image
    * @param nvjpegOutputType INPUT: One of NVJPEG_OUTPUT_* types
    * @param decodedChannels OUTPUT: List of decoded image channels. Memory must be pre-allocated. This method does not allocate memory for the output pointers.
    */
   private void decodeImage(BytePointer encodedImage,
                           long encodedImageSize,
                           NVJPEGImageInfo decodedImageInfo,
                           int nvjpegOutputType,
                           List<BytePointer> decodedChannels)
   {
      // Create NVJPEG image and allocate memory for decoded image
      int channelsToDecode = getNumberOfOutputChannels(nvjpegOutputType, decodedImageInfo);
      long[] decodedChannelSizes = getOutputChannelSizes(nvjpegOutputType, decodedImageInfo);
      long[] channelPitches = getOutputChannelPitches(nvjpegOutputType, decodedImageInfo);

      nvjpegImage_t nvjpegDecodedImage = new nvjpegImage_t();
      BytePointer[] decodedDeviceChannels = new BytePointer[channelsToDecode];

      for (int i = 0; i < channelsToDecode; ++i)
      {
         decodedDeviceChannels[i] = new BytePointer();
         checkCUDAError(cudaMallocAsync(decodedDeviceChannels[i], decodedChannelSizes[i], cudaStream));
         nvjpegDecodedImage.pitch(i, channelPitches[i]);
         nvjpegDecodedImage.channel(i, decodedDeviceChannels[i]);
      }

      if (decodedChannels.size() < channelsToDecode)
         throw new IllegalArgumentException("Too few pointers provided (must be greater than or equal to number of channels being decoded).");

      // Decode the image
      checkNVJPEGError(nvjpegDecode(nvjpegHandle, nvjpegState, encodedImage, encodedImageSize, nvjpegOutputType, nvjpegDecodedImage, cudaStream));

      // Copy data to output pointers
      for (int i = 0; i < channelsToDecode; ++i)
      {
         checkCUDAError(cudaMemcpyAsync(decodedChannels.get(i), decodedDeviceChannels[i], decodedChannelSizes[i], cudaMemcpyDefault, cudaStream));
      }
      checkCUDAError(cudaStreamSynchronize(cudaStream));

      // Free all memory
      for (int i = 0; i < channelsToDecode; ++i)
      {
         checkCUDAError(cudaFreeAsync(decodedDeviceChannels[i], cudaStream));
         decodedDeviceChannels[i].close();
      }
   }

   private int getNumberOfOutputChannels(int nvjpegOutputType, NVJPEGImageInfo imageInfo)
   {
      int channelsToDecode;
      switch (nvjpegOutputType)
      {
         case NVJPEG_OUTPUT_Y, NVJPEG_OUTPUT_RGBI, NVJPEG_OUTPUT_BGRI -> channelsToDecode = 1;
         case NVJPEG_OUTPUT_YUV, NVJPEG_OUTPUT_RGB, NVJPEG_OUTPUT_BGR -> channelsToDecode = 3;
         case NVJPEG_OUTPUT_UNCHANGED -> channelsToDecode = imageInfo.numberOfComponents();
         case NVJPEG_OUTPUT_UNCHANGEDI_U16 ->
               throw new UnsupportedOperationException("NVJPEG_OUTPUT_UNCHANGEDI_U16 is currently unsupported. Please feel free to implement it!");
         default -> throw new IllegalArgumentException("The provided nvjpegOutputType is unknown.");
      }

      return channelsToDecode;
   }

   private long[] getOutputChannelSizes(int nvjpegOutputType, NVJPEGImageInfo imageInfo)
   {
      int channelsToDecode = getNumberOfOutputChannels(nvjpegOutputType, imageInfo);

      if (imageInfo.widths.length < channelsToDecode || imageInfo.heights.length < channelsToDecode)
         throw new UnsupportedOperationException("Oops, we don't know how to deal with this :(");

      long[] decodedChannelSizes = new long[channelsToDecode];
      switch (nvjpegOutputType)
      {
         case NVJPEG_OUTPUT_Y -> decodedChannelSizes[0] = imageInfo.width(0) * imageInfo.height(0);
         case NVJPEG_OUTPUT_YUV, NVJPEG_OUTPUT_UNCHANGED ->
         {
            for (int i = 0; i < channelsToDecode; ++i)
               decodedChannelSizes[i] = imageInfo.width(i) * imageInfo.height(i);
         }
         case NVJPEG_OUTPUT_RGB, NVJPEG_OUTPUT_BGR ->
         {
            for (int i = 0; i < channelsToDecode; ++i)
               decodedChannelSizes[i] = imageInfo.width(0) * imageInfo.height(0);
         }
         case NVJPEG_OUTPUT_RGBI, NVJPEG_OUTPUT_BGRI -> decodedChannelSizes[0] = 3L * imageInfo.width(0) * imageInfo.height(0);
         case NVJPEG_OUTPUT_UNCHANGEDI_U16 ->
               throw new UnsupportedOperationException("NVJPEG_OUTPUT_UNCHANGEDI_U16 is currently unsupported. Please feel free to implement it!");
         default -> throw new IllegalArgumentException("The provided nvjpegOutputType is unknown.");
      }

      return decodedChannelSizes;
   }

   private long[] getOutputChannelPitches(int nvjpegOutputType, NVJPEGImageInfo imageInfo)
   {
      int channelsToDecode = getNumberOfOutputChannels(nvjpegOutputType, imageInfo);

      if (imageInfo.widths.length < channelsToDecode || imageInfo.heights.length < channelsToDecode)
         throw new UnsupportedOperationException("Oops, we don't know how to deal with this :(");

      long[] channelPitches = new long[channelsToDecode];
      switch (nvjpegOutputType)
      {
         case NVJPEG_OUTPUT_Y -> channelPitches[0] = imageInfo.width(0);
         case NVJPEG_OUTPUT_YUV, NVJPEG_OUTPUT_UNCHANGED ->
         {
            for (int i = 0; i < channelsToDecode; ++i)
               channelPitches[i] = imageInfo.width(i);
         }
         case NVJPEG_OUTPUT_RGB, NVJPEG_OUTPUT_BGR ->
         {
            for (int i = 0; i < channelsToDecode; ++i)
               channelPitches[i] = imageInfo.width(0);
         }
         case NVJPEG_OUTPUT_RGBI, NVJPEG_OUTPUT_BGRI -> channelPitches[0] = 3L * imageInfo.width(0);
         case NVJPEG_OUTPUT_UNCHANGEDI_U16 ->
               throw new UnsupportedOperationException("NVJPEG_OUTPUT_UNCHANGEDI_U16 is currently unsupported. Please feel free to implement it!");
         default -> throw new IllegalArgumentException("The provided nvjpegOutputType is unknown.");
      }

      return channelPitches;
   }

   private NVJPEGImageInfo getImageInfo(BytePointer encodedImage, long encodedImageSize)
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

   /**
    * Helper class for storing and passing around data obtained through nvjpegGetImageInfo().
    */
   private class NVJPEGImageInfo
   {
      private final int numberOfComponents;
      private final int[] subSamplingTypes;
      private final int[] widths;
      private final int[] heights;

      private NVJPEGImageInfo(int numberOfComponents, int[] subSamplingTypes, int[] widths, int[] heights)
      {
         this.numberOfComponents = numberOfComponents;
         this.subSamplingTypes = subSamplingTypes;
         this.widths = widths;
         this.heights = heights;
      }

      private int numberOfComponents()
      {
         return numberOfComponents;
      }

      private int subSamplingType(int channel)
      {
         return subSamplingTypes[channel];
      }

      private int width(int channel)
      {
         return widths[channel];
      }

      private int maxWidth()
      {
         int maxWidth = width(0);
         for (int i = 1; i < numberOfComponents; ++i)
            if (width(i) > maxWidth)
               maxWidth = width(i);
         return maxWidth;
      }

      private int height(int channel)
      {
         return heights[channel];
      }

      private int maxHeight()
      {
         int maxHeight = height(0);
         for (int i = 1; i < numberOfComponents; ++i)
            if (height(i) > maxHeight)
               maxHeight = height(i);
         return maxHeight;
      }
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
