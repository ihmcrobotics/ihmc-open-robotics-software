package us.ihmc.perception.cuda;

import org.apache.commons.lang3.NotImplementedException;
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
import org.bytedeco.opencv.opencv_core.GpuMat;
import org.bytedeco.opencv.opencv_core.GpuMatVector;
import org.bytedeco.opencv.opencv_core.Mat;
import org.bytedeco.opencv.opencv_core.MatVector;
import us.ihmc.perception.opencv.OpenCVTools;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import static org.bytedeco.cuda.global.cudart.*;
import static org.bytedeco.cuda.global.nvjpeg.*;
import static us.ihmc.perception.cuda.CUDATools.checkCUDAError;
import static us.ihmc.perception.cuda.CUDATools.checkNVJPEGError;

/**
 * Used for encoding images using CUDA.
 * Ensure that the computer has CUDA available before using this class. Most all Nvidia graphics cards should have CUDA available.
 * <p>
 * This class has been adapted from the SampleJpegEncoder of bytedeco:
 * <a href="https://github.com/bytedeco/javacpp-presets/blob/master/cuda/samples/SampleJpegEncoder.java">SampleJpegEncoder.java</a>
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
    *
    * @param quality value between 1 and 100 representing the quality of the jpeg image. 1 will result in the lowest quality, but highest compression.
    */
   public CUDAJPEGProcessor(int quality) throws UnsatisfiedLinkError
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
    * @param imageToEncode INPUT: the YUV I420 image (OpenCV type 8UC1)
    * @param encodedImage  OUTPUT: pointer to the jpeg image output
    */
   public void encodeYUVI420(Mat imageToEncode, BytePointer encodedImage)
   {
      encodeYUVI420(imageToEncode.data(), imageToEncode.cols(), imageToEncode.rows(), imageToEncode.step(), encodedImage);
   }

   /**
    * Encodes a YUV I420 image into jpeg.
    *
    * @param imageToEncode INPUT: the YUV I420 image (OpenCV type 8UC1)
    * @param encodedImage  OUTPUT: pointer to the jpeg image output
    */
   public void encodeYUVI420(GpuMat imageToEncode, BytePointer encodedImage)
   {
      encodeYUVI420(imageToEncode.data(), imageToEncode.cols(), imageToEncode.rows(), imageToEncode.step(), encodedImage);
   }

   private void encodeYUVI420(Pointer sourceImageData, int sourceWidth, int sourceHeight, long sourcePitch, BytePointer encodedImage)
   {
      int error;

      // Get some useful values
      int lumaHeight = (2 * sourceHeight) / 3;        // luma takes 2/3 of the total height
      int chromaHeight = sourceHeight - lumaHeight;   // chroma planes take rest of the height
      int chromaWidth = sourceWidth / 2;              // chroma planes each take half the width of the image

      // Declare and allocate CUDA pointers for each plane
      BytePointer yPlaneData = new BytePointer();
      BytePointer uPlaneData = new BytePointer();
      BytePointer vPlaneData = new BytePointer();

      CUDATools.mallocAsync(yPlaneData, (long) sourceWidth * lumaHeight, cudaStream);
      CUDATools.mallocAsync(uPlaneData, (long) chromaWidth * chromaHeight, cudaStream);
      CUDATools.mallocAsync(vPlaneData, (long) chromaWidth * chromaHeight, cudaStream);

      // Copy over each plane to
      Pointer planePointer = sourceImageData.getPointer();
      error = cudaMemcpy2DAsync(yPlaneData, sourceWidth, planePointer, sourcePitch, sourceWidth, lumaHeight, cudaMemcpyDefault, cudaStream);
      checkCUDAError(error);

      planePointer = planePointer.getPointer(sourcePitch * lumaHeight);
      error = cudaMemcpy2DAsync(uPlaneData, sourceWidth, planePointer, sourcePitch, sourceWidth, chromaHeight / 2, cudaMemcpyDefault, cudaStream);
      checkCUDAError(error);

      planePointer = planePointer.getPointer(sourcePitch * chromaHeight / 2);
      error = cudaMemcpy2DAsync(vPlaneData, sourceWidth, planePointer, sourcePitch, sourceWidth, chromaHeight / 2, cudaMemcpyDefault, cudaStream);
      checkCUDAError(error);

      // Create the nvjpeg image
      nvjpegImage_t nvjpegImage = new nvjpegImage_t();
      nvjpegImage.channel(0, yPlaneData).pitch(0, sourceWidth);   // Y
      nvjpegImage.channel(1, uPlaneData).pitch(1, chromaWidth);   // U
      nvjpegImage.channel(2, vPlaneData).pitch(2, chromaWidth);   // V

      // Encode the image
      encodeYUV(nvjpegImage, sourceWidth, lumaHeight, NVJPEG_CSS_420, encodedImage);

      // Free resources
      checkCUDAError(cudaFreeAsync(yPlaneData, cudaStream));
      checkCUDAError(cudaFreeAsync(uPlaneData, cudaStream));
      checkCUDAError(cudaFreeAsync(vPlaneData, cudaStream));
      yPlaneData.close();
      uPlaneData.close();
      vPlaneData.close();
      planePointer.close();
      nvjpegImage.close();
   }

   /**
    * Encodes a YUV 444 image into jpeg.
    *
    * @param imageToEncode INPUT: the YUV 444 image (OpenCV type 8UC3)
    * @param encodedImage  OUTPUT: pointer to the jpeg image output
    */
   public void encodeYUV444(Mat imageToEncode, BytePointer encodedImage)
   {
      // Split the source image into its planes
      MatVector sourceImagePlanes = new MatVector();
      opencv_core.split(imageToEncode, sourceImagePlanes);

      // Get the pointers to the planes
      Pointer[] planePointers = new Pointer[(int) sourceImagePlanes.size()];
      long[] planePitches = new long[(int) sourceImagePlanes.size()];
      for (int i = 0; i < sourceImagePlanes.size(); ++i)
      {
         planePointers[i] = sourceImagePlanes.get(i).data();
         planePitches[i] = sourceImagePlanes.get(i).step();
      }

      // Encode the image
      encodeYUV444(planePointers, planePitches, imageToEncode.cols(), imageToEncode.rows(), encodedImage);

      // Free resources
      for (int i = 0; i < sourceImagePlanes.size(); ++i)
         planePointers[i].close();
      sourceImagePlanes.close();
   }

   /**
    * Encodes a YUV 444 image into jpeg.
    *
    * @param imageToEncode INPUT: the YUV 444 image (OpenCV type 8UC3)
    * @param encodedImage  OUTPUT: pointer to the jpeg image output
    */
   public void encodeYUV444(GpuMat imageToEncode, BytePointer encodedImage)
   {
      // Split the source image into its planes
      GpuMatVector sourceImagePlanes = new GpuMatVector();
      opencv_cudaarithm.split(imageToEncode, sourceImagePlanes);

      // Get the pointers to the planes
      Pointer[] planePointers = new Pointer[(int) sourceImagePlanes.size()];
      long[] planePitches = new long[(int) sourceImagePlanes.size()];
      for (int i = 0; i < sourceImagePlanes.size(); ++i)
      {
         planePointers[i] = sourceImagePlanes.get(i).data();
         planePitches[i] = sourceImagePlanes.get(i).step();
      }

      // Encode the image
      encodeYUV444(planePointers, planePitches, imageToEncode.cols(), imageToEncode.rows(), encodedImage);

      // Free resources
      for (int i = 0; i < sourceImagePlanes.size(); ++i)
         planePointers[i].close();
      sourceImagePlanes.close();
   }

   private void encodeYUV444(Pointer[] sourceImagePlanes, long[] planePitches, int width, int height, BytePointer encodedImage)
   {
      // Create the nvjpeg image
      nvjpegImage_t nvjpegImage = new nvjpegImage_t();

      // Copy planes data into it
      for (int i = 0; i < 3; ++i)
      {
         BytePointer plane = new BytePointer();
         CUDATools.mallocAsync(plane, (long) width * height, cudaStream);
         checkCUDAError(cudaMemcpy2DAsync(plane, width, sourceImagePlanes[i], planePitches[i], width, height, cudaMemcpyDefault, cudaStream));
         nvjpegImage.channel(i, plane).pitch(i, width);
      }

      // Encode the iamge
      encodeYUV(nvjpegImage, width, height, NVJPEG_CSS_444, encodedImage);

      // Free resources
      for (int i = 0; i < 3; ++i)
         checkCUDAError(cudaFree(nvjpegImage.channel(i)));
      nvjpegImage.close();
   }

   /**
    * Encodes a gray image into jpeg.
    *
    * @param imageToEncode INPUT: the gray image (OpenCV type 8UC1)
    * @param encodedImage  OUTPUT: pointer to the jpeg image output
    */
   public void encodeGray(Mat imageToEncode, BytePointer encodedImage)
   {
      encodeGray(imageToEncode.data(), imageToEncode.step(), imageToEncode.cols(), imageToEncode.rows(), encodedImage);
   }

   /**
    * Encodes a gray image into jpeg.
    *
    * @param imageToEncode INPUT: the gray image (OpenCV type 8UC1)
    * @param encodedImage  OUTPUT: pointer to the jpeg image output
    */
   public void encodeGray(GpuMat imageToEncode, BytePointer encodedImage)
   {
      encodeGray(imageToEncode.data(), imageToEncode.step(), imageToEncode.cols(), imageToEncode.rows(), encodedImage);
   }

   private void encodeGray(Pointer sourceImageData, long pitch, int width, int height, BytePointer encodedImage)
   {
      // Copy data over to CUDA memory
      BytePointer cudaData = new BytePointer();
      CUDATools.mallocAsync(cudaData, (long) width * height, cudaStream);
      checkCUDAError(cudaMemcpy2DAsync(cudaData, width, sourceImageData, pitch, width, height, cudaMemcpyDefault, cudaStream));

      // Create nvjpeg image
      nvjpegImage_t nvjpegImage = new nvjpegImage_t();
      nvjpegImage.channel(0, cudaData).pitch(0, width);

      // Encode the image
      encodeYUV(nvjpegImage, width, height, NVJPEG_CSS_GRAY, encodedImage);

      // Free resources
      cudaFreeAsync(cudaData, cudaStream);
      cudaData.close();
      nvjpegImage.close();
   }

   private void encodeYUV(nvjpegImage_t imageToEncode, int width, int height, int chromaSubSampling, BytePointer encodedImage)
   {
      // Set sampling factor
      checkNVJPEGError(nvjpegEncoderParamsSetSamplingFactors(encoderParameters, chromaSubSampling, cudaStream));

      // Encode the image
      checkNVJPEGError(nvjpegEncodeYUV(nvjpegHandle, encoderState, encoderParameters, imageToEncode, chromaSubSampling, width, height, cudaStream));

      // Get compressed image size
      SizeTPointer jpegSize = new SizeTPointer(1);
      checkNVJPEGError(nvjpegEncodeRetrieveBitstream(nvjpegHandle, encoderState, (BytePointer) null, jpegSize, cudaStream));

      // Retrieve bitstream
      encodedImage.limit(jpegSize.get());
      checkNVJPEGError(nvjpegEncodeRetrieveBitstream(nvjpegHandle, encoderState, encodedImage, jpegSize, cudaStream));

      checkCUDAError(cudaStreamSynchronize(cudaStream));

      // Close everything
      jpegSize.close();
   }

   /**
    * Encodes a BGR image into jpeg
    *
    * @param imageToEncode INPUT: the BGR image (OpenCV type 8UC3)
    * @param encodedImage  OUTPUT: pointer to the jpeg image output
    */
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

   /**
    * Encodes a BGR image into jpeg
    *
    * @param imageToEncode INPUT: the BGR image (OpenCV type 8UC3)
    * @param encodedImage  OUTPUT: pointer to the jpeg image output
    */
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

   /**
    * Encodes a RGB image into jpeg
    *
    * @param imageToEncode INPUT: the RGB image (OpenCV type 8UC3)
    * @param encodedImage  OUTPUT: pointer to the jpeg image output
    */
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

   /**
    * Encodes a RGB image into jpeg
    *
    * @param imageToEncode INPUT: the RGB image (OpenCV type 8UC3)
    * @param encodedImage  OUTPUT: pointer to the jpeg image output
    */
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

   /**
    * Encodes an interleaved image into jpeg
    *
    * @param sourceImageData        the interleaved image data to encode
    * @param imageWidth             width of the source image
    * @param imageHeight            height of the source image
    * @param imagePitch             pitch (aka step in OpenCV land) of the source image
    * @param interleavedInputFormat either NVJPEG_INPUT_BGRI or NVJPEG_INPUT_RGBI
    * @param encodedImage           output pointer for encoded data
    */
   public void encodeInterleaved(BytePointer sourceImageData,
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
      checkCUDAError(cudaMemcpy2DAsync(deviceImagePointer, rowSize, sourceImageData, imagePitch, rowSize, imageHeight, cudaMemcpyDefault, cudaStream));

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
    *
    * @param encodedImage     INPUT: a jpeg encoded image.
    * @param encodedImageSize INPUT: size, in bytes, of the encoded image.
    * @param decodedImage     OUTPUT: the decoded image.
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
    *
    * @param encodedImage     INPUT: a jpeg encoded image.
    * @param encodedImageSize INPUT: size, in bytes, of the encoded image.
    * @param decodedImage     OUTPUT: the decoded image.
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
    * Decodes a jpeg encoded image to a YUV format.
    * The output YUV format will match the format of the encoded image data.
    * Currently, this may be YUV444 or YUV I420.
    *
    * @param encodedImage     INPUT: a jpeg encoded image.
    * @param encodedImageSize INPUT: size, in bytes, of the encoded image.
    * @param decodedImage     OUTPUT: the decoded image.
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
           MatVector planesVector = new MatVector(yPlane, uPlane, vPlane);
           Mat yuvCombinedImage = new Mat())
      {
         switch (imageInfo.subSamplingType(0))
         {
            case NVJPEG_CSS_420 ->
            {
               // Create the I420 image with correct size
               yuvCombinedImage.create(imageInfo.height(0) + imageInfo.height(1), imageInfo.width(0), opencv_core.CV_8UC1);
               // Pack the YUV planes into their respective locations
               Pointer.memcpy(yuvCombinedImage.ptr(), yPlane.ptr(), OpenCVTools.dataSize(yPlane));
               Pointer.memcpy(yuvCombinedImage.ptr(imageInfo.height(0), 0), uPlane.ptr(), OpenCVTools.dataSize(uPlane));
               Pointer.memcpy(yuvCombinedImage.ptr(imageInfo.height(0) + imageInfo.height(1) / 2, 0), vPlane.ptr(), OpenCVTools.dataSize(vPlane));
            }
            case NVJPEG_CSS_444 -> opencv_core.merge(planesVector, yuvCombinedImage);
            default -> throw new NotImplementedException("Tomasz didn't have enough time to write code for this YUV format. Maybe you could?");
         }

         // Copy the YUV image into the output image
         yuvCombinedImage.copyTo(decodedImage);
      }

      // Free all memory
      for (int i = 0; i < numberOfDecodedChannels; ++i)
      {
         checkCUDAError(cudaFreeHost(decodedChannels.get(i)));
         decodedChannels.get(i).close();
      }
   }

   /**
    * Decodes a jpeg encoded image to gray format.
    *
    * @param encodedImage     INPUT: a jpeg encoded image.
    * @param encodedImageSize INPUT: size, in bytes, of the encoded image.
    * @param decodedImage     OUTPUT: the decoded image.
    */
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

   /**
    * Decodes a jpeg encoded image to gray format.
    *
    * @param encodedImage     INPUT: a jpeg encoded image.
    * @param encodedImageSize INPUT: size, in bytes, of the encoded image.
    * @param decodedImage     OUTPUT: the decoded image.
    */
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
    * Decodes a jpeg encoded image into the specified output type.
    *
    * @param encodedImage     INPUT: An encoded multi-channel image.
    * @param encodedImageSize INPUT: Number of bytes of the encoded image.
    * @param decodedImageInfo INPUT: {@link NVJPEGImageInfo} about the passed in image
    * @param nvjpegOutputType INPUT: One of NVJPEG_OUTPUT_* types
    * @param decodedChannels  OUTPUT: List of decoded image channels. Memory must be pre-allocated. This method does not allocate memory for the output
    *                         pointers.
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
         case NVJPEG_OUTPUT_Y -> decodedChannelSizes[0] = (long) imageInfo.width(0) * imageInfo.height(0);
         case NVJPEG_OUTPUT_YUV, NVJPEG_OUTPUT_UNCHANGED ->
         {
            for (int i = 0; i < channelsToDecode; ++i)
               decodedChannelSizes[i] = (long) imageInfo.width(i) * imageInfo.height(i);
         }
         case NVJPEG_OUTPUT_RGB, NVJPEG_OUTPUT_BGR -> Arrays.fill(decodedChannelSizes, (long) imageInfo.width(0) * imageInfo.height(0));
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
         case NVJPEG_OUTPUT_RGB, NVJPEG_OUTPUT_BGR -> Arrays.fill(channelPitches, imageInfo.width(0));
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
   private record NVJPEGImageInfo(int numberOfComponents, int[] subSamplingTypes, int[] widths, int[] heights)
   {
      private int subSamplingType(int channel)
      {
         return subSamplingTypes[channel];
      }

      private int width(int channel)
      {
         return widths[channel];
      }

      private int height(int channel)
      {
         return heights[channel];
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
