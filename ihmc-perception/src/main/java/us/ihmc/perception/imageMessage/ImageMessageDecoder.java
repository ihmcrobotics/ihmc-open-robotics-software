package us.ihmc.perception.imageMessage;

import org.bytedeco.javacpp.BytePointer;
import org.bytedeco.opencv.global.opencv_cudaimgproc;
import org.bytedeco.opencv.global.opencv_imgcodecs;
import org.bytedeco.opencv.global.opencv_imgproc;
import org.bytedeco.opencv.opencv_core.GpuMat;
import org.bytedeco.opencv.opencv_core.Mat;
import perception_msgs.ImageMessage;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.log.LogTools;
import us.ihmc.perception.CameraModel;
import us.ihmc.perception.RawImage;
import us.ihmc.perception.cuda.CUDAJPEGProcessor;
import us.ihmc.perception.cuda.CUDATools;
import us.ihmc.sensors.CameraIntrinsics;

import java.time.Instant;

public class ImageMessageDecoder
{
   private final ImageMessageDecompressionInput messageDataExtractor = new ImageMessageDecompressionInput();

   private CUDAJPEGProcessor cudaJpegDecoder = null;

   private PixelFormat lastImagePixelFormat = null;

   public ImageMessageDecoder()
   {
      if (CUDATools.hasCUDADevice() && CUDATools.hasNVJPEG())
         cudaJpegDecoder = new CUDAJPEGProcessor();
   }

   public RawImage decodeMessageCPU(ImageMessage messageToDecode)
   {
      Mat image = new Mat();
      decodeMessage(messageToDecode, image);

      CameraIntrinsics intrinsics = new CameraIntrinsics();
      intrinsics.setWidth(messageToDecode.getImageWidth());
      intrinsics.setHeight(messageToDecode.getImageHeight());
      intrinsics.setFx(messageToDecode.getFocalLengthXPixels());
      intrinsics.setFy(messageToDecode.getFocalLengthYPixels());
      intrinsics.setCx(messageToDecode.getPrincipalPointXPixels());
      intrinsics.setCy(messageToDecode.getPrincipalPointYPixels());

      CameraModel cameraModel = CameraModel.fromByte(messageToDecode.getCameraModel());
      FramePose3D sensorPose = new FramePose3D(ReferenceFrame.getWorldFrame(),
                                               messageToDecode.getPosition().getPoint(),
                                               messageToDecode.getOrientation().getQuaternion());
      Instant acquisitionTime = MessageTools.toInstant(messageToDecode.getAcquisitionTime());
      long sequenceNumber = messageToDecode.getSequenceNumber();
      float depthDiscretization = messageToDecode.getDepthDiscretization();

      return new RawImage(image, null, getDecodedImagePixelFormat(), intrinsics, cameraModel, sensorPose, acquisitionTime, sequenceNumber, depthDiscretization);
   }

   /**
    * Decode the image contained within the {@code messageToDecode}, and pack it into the {@code imageToPack}.
    * Use {@link #getDecodedImagePixelFormat()} to get the pixel format of the decoded image.
    * @param messageToDecode The message containing image data
    * @param imageToPack The image into which the decoded image will be packed.
    */
   public void decodeMessage(ImageMessage messageToDecode, Mat imageToPack)
   {
      if (!hasValidMessageMetadata(messageToDecode))
         return;

      resizeToMessageDimensions(messageToDecode, imageToPack);
      messageDataExtractor.extract(messageToDecode);

      lastImagePixelFormat = PixelFormat.fromImageMessage(messageToDecode);

      switch (CompressionType.fromImageMessage(messageToDecode))
      {
         case JPEG, PNG ->
         {
            opencv_imgcodecs.imdecode(messageDataExtractor.getInputMat(), opencv_imgcodecs.IMREAD_UNCHANGED, imageToPack);
            // RGBA or BGRA will lose the alpha channel in jpeg encoding, so we give it back
            if (lastImagePixelFormat.elementsPerPixel == 4 && imageToPack.channels() == 3)
               opencv_imgproc.cvtColor(imageToPack, imageToPack, opencv_imgproc.COLOR_BGR2BGRA);
         }
         case NVJPEG ->
         {
            if (cudaJpegDecoder != null)
            {  // Use CUDA acceleration if available
               BytePointer encodedData = messageDataExtractor.getInputPointer();
               cudaJpegDecoder.decodeToBGR(encodedData, encodedData.limit(), imageToPack);
            }
            else
            {  // Otherwise use OpenCV
               opencv_imgcodecs.imdecode(messageDataExtractor.getInputMat(), opencv_imgcodecs.IMREAD_UNCHANGED, imageToPack);
            }
            lastImagePixelFormat = PixelFormat.BGR8;
         }
         case UNCOMPRESSED ->
         {
            if (imageToPack.elemSize() != lastImagePixelFormat.bytesPerPixel)
            {
               Mat correctSizedMat = new Mat(messageToDecode.getImageHeight(), messageToDecode.getImageWidth(), lastImagePixelFormat.toOpenCVType());
               correctSizedMat.copyTo(imageToPack);
               correctSizedMat.close();
            }
            imageToPack.data(messageDataExtractor.getInputPointer().position(0));
         }
         case UNKNOWN -> LogTools.warn("Skipping ImageMessage decode with unknown compression type.");
      }
   }

   /**
    * Decode the image contained within the {@code messageToDecode}, and pack it into the {@code imageToPack}.
    * Use {@link #getDecodedImagePixelFormat()} to get the pixel format of the decoded image.
    * @param messageToDecode The message containing image data
    * @param imageToPack The image into which the decoded image will be packed.
    */
   public void decodeMessage(ImageMessage messageToDecode, GpuMat imageToPack)
   {
      if (!hasValidMessageMetadata(messageToDecode))
         return;

      resizeToMessageDimensions(messageToDecode, imageToPack);
      messageDataExtractor.extract(messageToDecode);

      lastImagePixelFormat = PixelFormat.fromImageMessage(messageToDecode);

      switch (CompressionType.fromImageMessage(messageToDecode))
      {
         case JPEG ->
         {
            // NVJPEG and JPEG and not compatible, but with some hacks we can get them to work
            if (lastImagePixelFormat.elementsPerPixel == 1)
            {  // OpenCV seems to compress 1 channel images (including YUV_I420) as gray scale
               cudaJpegDecoder.decodeToGray(messageDataExtractor.getInputPointer(), messageDataExtractor.getInputPointer().limit(), imageToPack);
            }
            else
            {  // NVJPEG does not recognize the color format of OpenCV compressed jpegs, so RGB jpegs can be decoded as BGR
               cudaJpegDecoder.decodeToBGR(messageDataExtractor.getInputPointer(), messageDataExtractor.getInputPointer().limit(), imageToPack);
               if (lastImagePixelFormat.elementsPerPixel == 4 && imageToPack.channels() == 3)
                  opencv_cudaimgproc.cvtColor(imageToPack, imageToPack, opencv_imgproc.COLOR_BGR2BGRA);
            }
         }
         case PNG ->
         {
            Mat decompressedImage = new Mat();
            opencv_imgcodecs.imdecode(messageDataExtractor.getInputMat(), opencv_imgcodecs.IMREAD_UNCHANGED, decompressedImage);
            imageToPack.upload(decompressedImage);
            decompressedImage.close();
         }
         case NVJPEG ->
         {
            BytePointer encodedData = messageDataExtractor.getInputPointer();
            cudaJpegDecoder.decodeToBGR(encodedData, encodedData.limit(), imageToPack);
            lastImagePixelFormat = PixelFormat.BGR8;
         }
         case UNCOMPRESSED ->
         {
            Mat cpuImage = new Mat(imageToPack.size(), lastImagePixelFormat.toOpenCVType(), messageDataExtractor.getInputPointer());
            imageToPack.upload(cpuImage);
            cpuImage.close();
         }
         case UNKNOWN -> LogTools.warn("Skipping ImageMessage decode with unknown compression type.");
      }
   }

   /**
    * Decode the image contained within the {@code messageToDecode}, converts it into RGBA, and pack it into the {@code imageToPack}.
    * If the decoded image cannot be converted to RGBA, it will remain in its original pixel format.
    * @param messageToDecode The message containing image data
    * @param imageToPack The image into which the decoded RGBA image will be packed.
    */
   public void decodeMessageToRGBA(ImageMessage messageToDecode, Mat imageToPack)
   {
      decodeMessage(messageToDecode, imageToPack);
      if (lastImagePixelFormat.convertToRGBA(imageToPack, imageToPack))
         lastImagePixelFormat = PixelFormat.RGBA8;
      else
         LogTools.warn("Attempted conversion from {} to {} failed.", lastImagePixelFormat, PixelFormat.RGBA8);
   }

   /**
    * Decode the image contained within the {@code messageToDecode}, converts it into RGBA, and pack it into the {@code imageToPack}.
    * If the decoded image cannot be converted to RGBA, it will remain in its original pixel format.
    * @param messageToDecode The message containing image data
    * @param imageToPack The image into which the decoded RGBA image will be packed.
    */
   public void decodeMessageToRGBA(ImageMessage messageToDecode, GpuMat imageToPack)
   {
      decodeMessage(messageToDecode, imageToPack);
      if (lastImagePixelFormat.convertToRGBA(imageToPack, imageToPack))
         lastImagePixelFormat = PixelFormat.RGBA8;
      else
         LogTools.warn("Attempted conversion from {} to {} failed.", lastImagePixelFormat, PixelFormat.RGBA8);
   }

   /**
    * @return The pixel format of the last decoded image.
    */
   public PixelFormat getDecodedImagePixelFormat()
   {
      return lastImagePixelFormat;
   }

   public void destroy()
   {
      if (cudaJpegDecoder != null)
         cudaJpegDecoder.destroy();

      messageDataExtractor.destroy();
   }

   private void resizeToMessageDimensions(ImageMessage imageMessage, Mat imageToResize)
   {
      PixelFormat pixelFormat = PixelFormat.fromByte(imageMessage.getPixelFormat());
      imageToResize.create(imageMessage.getImageHeight(), imageMessage.getImageWidth(), pixelFormat.toOpenCVType());
   }

   private void resizeToMessageDimensions(ImageMessage imageMessage, GpuMat imageToResize)
   {
      PixelFormat pixelFormat = PixelFormat.fromByte(imageMessage.getPixelFormat());
      imageToResize.create(imageMessage.getImageHeight(), imageMessage.getImageWidth(), pixelFormat.toOpenCVType());
   }

   private static boolean hasValidMessageMetadata(ImageMessage imageMessage)
   {
      if (PixelFormat.fromByte(imageMessage.getPixelFormat()) == PixelFormat.UNKNOWN)
      {
         LogTools.warn("ImageMessage has invalid pixel_format byte: {}", imageMessage.getPixelFormat() & 0xFF);
         return false;
      }

      if (CompressionType.fromByte(imageMessage.getCompressionType()) == CompressionType.UNKNOWN)
      {
         LogTools.warn("ImageMessage has invalid compression_type byte: {}", imageMessage.getCompressionType() & 0xFF);
         return false;
      }

      return true;
   }
}
