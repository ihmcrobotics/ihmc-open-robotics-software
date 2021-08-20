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

import java.nio.ByteBuffer;
import java.util.Arrays;
import java.util.Vector;

public class CvImageTools
{
   public static void backMatWithNettyImageBuffer(sensor_msgs.Image ros1Image, Mat inputImageMat)
   {
      ChannelBuffer nettyImageData = ros1Image.getData();
      ByteBuffer dataByteBuffer = nettyImageData.toByteBuffer();
      int arrayOffset = nettyImageData.arrayOffset();
      dataByteBuffer.position(arrayOffset);
      ByteBuffer offsetByteBuffer = dataByteBuffer.slice();
      System.out.println(dataByteBuffer.isDirect());

      BytePointer imageDataPointer = new BytePointer(offsetByteBuffer);
      inputImageMat.data(imageDataPointer);
   }

   public static CvImage toCvCopy(final Image source) throws Exception
   {
      return toCvCopyImpl(matFromImage(source), source.getHeader(), source.getEncoding(), "");
   }

   public static CvImage toCvCopy(final Image source, final String dst_encoding) throws Exception
   {
      return toCvCopyImpl(matFromImage(source), source.getHeader(), source.getEncoding(), dst_encoding);
   }

   public static CvImage toCvCopy(final CompressedImage source) throws Exception
   {
      return toCvCopyImpl(matFromImage(source), source.getHeader(), ImageEncodings.BGR8, "");
   }

   public static CvImage toCvCopy(final CompressedImage source, final String dst_encoding) throws Exception
   {
      return toCvCopyImpl(matFromImage(source), source.getHeader(), ImageEncodings.BGR8, dst_encoding);
   }

   public static CvImage cvtColor(final CvImage source, String encoding) throws Exception
   {
      return toCvCopyImpl(source.image, source.header, source.encoding, encoding);
   }

   public static CvImage toCvCopyImpl(final Mat source, final Header sourceHeader, final String sourceEncoding, final String destinationEncoding) throws Exception
   {
      /// @todo Handle endianness - e.g. 16-bit dc1394 camera images are big-endian
      /// Languages such as Java manage this for you so that Java code can run on any platform and programmers do not have to manage byte ordering.
      /// from http://www.yolinux.com/TUTORIALS/Endian-Byte-Order.html
      /// need to check if it true in our case with this cameras.

      // Copy metadata
      CvImage cvImage = new CvImage();
      cvImage.header = sourceHeader;

      // Copy to new buffer if same encoding requested
      if (destinationEncoding.isEmpty() || destinationEncoding.equals(sourceEncoding))
      {
         cvImage.encoding = sourceEncoding;
         source.copyTo(cvImage.image);
      }
      else
      {
         // Convert the source data to the desired encoding
         final Vector<Integer> conversionCodes = ImEncoding.getConversionCode(sourceEncoding, destinationEncoding);
         Mat image1 = source;
         Mat image2 = new Mat();

         for (int i = 0; i < conversionCodes.size(); ++i)
         {
            int conversionCode = conversionCodes.get(i);
            if (conversionCode == ImEncoding.SAME_FORMAT)
            {
               //convert from Same number of channels, but different bit depth

               //double alpha = 1.0;
               int sourceDepth = ImageEncodings.bitDepth(sourceEncoding);
               int destinationDepth = ImageEncodings.bitDepth(destinationEncoding);
               // Do scaling between CV_8U [0,255] and CV_16U [0,65535] images.
               //from http://www.rubydoc.info/github/ruby-opencv/ruby-opencv/OpenCV/CvMat
               //from http://docs.opencv.org/modules/core/doc/basic_structures.html
               //TODO: check which value default for beta is ok.
               int beta = 0;
               int image2Type = opencv_core.CV_MAKETYPE(opencv_core.CV_MAT_DEPTH(ImEncoding.getCvType(destinationEncoding)), image1.channels());
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
         cvImage.image = image2;
         cvImage.encoding = destinationEncoding;
      }
      return cvImage;
   }

   public static Mat matFromImage(final Image source) throws Exception
   {
      byte[] imageInBytes = source.getData().array();
      imageInBytes = Arrays.copyOfRange(imageInBytes, source.getData().arrayOffset(), imageInBytes.length);
      String encoding = source.getEncoding().toUpperCase();
      Mat cvImage = new Mat(source.getHeight(), source.getWidth(), ImEncoding.getCvType(encoding));

      BytePointer bytePointer = new BytePointer(imageInBytes);
      cvImage = cvImage.data(bytePointer);
      return cvImage;
   }

   public static Mat matFromImage(final CompressedImage source) throws Exception
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
