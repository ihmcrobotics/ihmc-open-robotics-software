package us.ihmc.perception;

import org.bytedeco.javacpp.BytePointer;
import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.global.opencv_imgcodecs;
import org.bytedeco.opencv.global.opencv_imgproc;
import org.bytedeco.opencv.opencv_core.Mat;
import org.jboss.netty.buffer.ChannelBuffer;
import sensor_msgs.CompressedImage;
import sensor_msgs.Image;
import std_msgs.Header;
import us.ihmc.utilities.ros.RosTools;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.util.Arrays;
import java.util.Vector;

public class ROSOpenCVTools
{
   public static ByteBuffer getAsDirectByteBuffer(ChannelBuffer channelBuffer)
   {
      ByteBuffer slicedBuffer = RosTools.sliceNettyBuffer(channelBuffer);
      ByteBuffer slicedDirectBuffer;
      if (!slicedBuffer.isDirect()) // TODO: How to get Netty to receive via direct buffers?
      {
         slicedDirectBuffer = ByteBuffer.allocateDirect(slicedBuffer.capacity());
         slicedDirectBuffer.order(ByteOrder.nativeOrder());
         slicedBuffer.rewind();
         slicedDirectBuffer.put(slicedBuffer);
         slicedDirectBuffer.rewind();
      }
      else
      {
         slicedDirectBuffer = slicedBuffer;
      }

      return slicedDirectBuffer;
   }

   public static void backMatWithNettyBuffer(Mat mat, ChannelBuffer channelBuffer)
   {
      BytePointer imageDataPointer = new BytePointer(getAsDirectByteBuffer(channelBuffer));
      mat.data(imageDataPointer);
   }

   public static ROSOpenCVImage toCvCopy(final Image source) throws Exception
   {
      return toCvCopyImpl(matFromImageAndCopyData(source), source.getHeader(), source.getEncoding(), "");
   }

   public static ROSOpenCVImage toCvCopy(final Image source, final String dst_encoding) throws Exception
   {
      return toCvCopyImpl(matFromImageAndCopyData(source), source.getHeader(), source.getEncoding(), dst_encoding);
   }

   public static ROSOpenCVImage toCvCopy(final CompressedImage source) throws Exception
   {
      return toCvCopyImpl(matFromImageAndCopyData(source), source.getHeader(), ImageEncodingTools.BGR8, "");
   }

   public static ROSOpenCVImage toCvCopy(final CompressedImage source, final String dst_encoding) throws Exception
   {
      return toCvCopyImpl(matFromImageAndCopyData(source), source.getHeader(), ImageEncodingTools.BGR8, dst_encoding);
   }

   public static ROSOpenCVImage cvtColor(final ROSOpenCVImage source, String encoding) throws Exception
   {
      return toCvCopyImpl(source.image, source.header, source.encoding, encoding);
   }

   public static ROSOpenCVImage toCvCopyImpl(final Mat source, final Header sourceHeader, final String sourceEncoding, final String destinationEncoding) throws Exception
   {
      /// @todo Handle endianness - e.g. 16-bit dc1394 camera images are big-endian
      /// Languages such as Java manage this for you so that Java code can run on any platform and programmers do not have to manage byte ordering.
      /// from http://www.yolinux.com/TUTORIALS/Endian-Byte-Order.html
      /// need to check if it true in our case with this cameras.

      // Copy metadata
      ROSOpenCVImage ROSOpenCVImage = new ROSOpenCVImage();
      ROSOpenCVImage.header = sourceHeader;

      // Copy to new buffer if same encoding requested
      if (destinationEncoding.isEmpty() || destinationEncoding.equals(sourceEncoding))
      {
         ROSOpenCVImage.encoding = sourceEncoding;
         source.copyTo(ROSOpenCVImage.image);
      }
      else
      {
         // Convert the source data to the desired encoding
         final Vector<Integer> conversionCodes = ImageEncodingTools.getColorConversionCode(sourceEncoding, destinationEncoding);
         Mat image1 = source;
         Mat image2 = new Mat();

         for (int i = 0; i < conversionCodes.size(); ++i)
         {
            int conversionCode = conversionCodes.get(i);
            if (conversionCode == ImageEncodingTools.SAME_FORMAT)
            {
               //convert from Same number of channels, but different bit depth

               //double alpha = 1.0;
               int sourceDepth = ImageEncodingTools.bitDepth(sourceEncoding);
               int destinationDepth = ImageEncodingTools.bitDepth(destinationEncoding);
               // Do scaling between CV_8U [0,255] and CV_16U [0,65535] images.
               //from http://www.rubydoc.info/github/ruby-opencv/ruby-opencv/OpenCV/CvMat
               //from http://docs.opencv.org/modules/core/doc/basic_structures.html
               //TODO: check which value default for beta is ok.
               int beta = 0;
               int image2Type = opencv_core.CV_MAKETYPE(opencv_core.CV_MAT_DEPTH(ImageEncodingTools.getCvType(destinationEncoding)), image1.channels());
               if (sourceDepth == 8 && destinationDepth == 16)
                  image1.convertTo(image2, image2Type, 65535. / 255., beta);
               else if (sourceDepth == 16 && destinationDepth == 8)
                  image1.convertTo(image2, image2Type, 255. / 65535., beta);
               else
                  image1.convertTo(image2, image2Type);
            }
            else
            {
               // Perform color conversion
               opencv_imgproc.cvtColor(image1, image2, conversionCodes.get(0));
            }
            image1 = image2;
         }
         ROSOpenCVImage.image = image2;
         ROSOpenCVImage.encoding = destinationEncoding;
      }
      return ROSOpenCVImage;
   }

   public static void convert()
   {

   }

   public static Mat matFromImage(final Image source)
   {
      String encoding = source.getEncoding().toUpperCase();
      return new Mat(source.getHeight(), source.getWidth(), ImageEncodingTools.getCvType(encoding));
   }

   public static Mat matFromImage(final CompressedImage source)
   {
      ChannelBuffer data = source.getData();
      byte[] imageInBytes = data.array();
      imageInBytes = Arrays.copyOfRange(imageInBytes, source.getData().arrayOffset(), imageInBytes.length);

      int length = source.getData().array().length - source.getData().arrayOffset();
      //from http://stackoverflow.com/questions/23202130/android-convert-byte-array-from-camera-api-to-color-mat-object-opencv
      Mat cvImage = new Mat(1, imageInBytes.length, opencv_core.CV_8UC1);
      BytePointer bytePointer = new BytePointer(imageInBytes);
      cvImage = cvImage.data(bytePointer);

      return opencv_imgcodecs.imdecode(cvImage, opencv_imgcodecs.IMREAD_ANYCOLOR);
   }

   public static Mat matFromImageAndCopyData(final Image source)
   {
      byte[] imageInBytes = source.getData().array();
      imageInBytes = Arrays.copyOfRange(imageInBytes, source.getData().arrayOffset(), imageInBytes.length);
      Mat cvImage = matFromImage(source);

      BytePointer bytePointer = new BytePointer(imageInBytes);
      cvImage = cvImage.data(bytePointer);
      return cvImage;
   }

   public static Mat matFromImageAndCopyData(final CompressedImage source)
   {
      ChannelBuffer data = source.getData();
      byte[] imageInBytes = data.array();
      imageInBytes = Arrays.copyOfRange(imageInBytes, source.getData().arrayOffset(), imageInBytes.length);
      //from http://stackoverflow.com/questions/23202130/android-convert-byte-array-from-camera-api-to-color-mat-object-opencv
      Mat cvImage = new Mat(1, imageInBytes.length, opencv_core.CV_8UC1);
      BytePointer bytePointer = new BytePointer(imageInBytes);
      cvImage = cvImage.data(bytePointer);

      return opencv_imgcodecs.imdecode(cvImage, opencv_imgcodecs.IMREAD_ANYCOLOR);
   }
}
