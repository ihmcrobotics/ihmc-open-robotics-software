/*
 * Copyright (c) 2015, Tal Regev
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Android Sensors Driver nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

package us.ihmc.perception;

import org.bytedeco.javacpp.BytePointer;
import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.global.opencv_imgcodecs;
import org.bytedeco.opencv.global.opencv_imgproc;
import org.bytedeco.opencv.opencv_core.*;
import org.jboss.netty.buffer.ChannelBuffer;
import org.jboss.netty.buffer.ChannelBufferOutputStream;
import org.ros.internal.message.MessageBuffers;

import java.io.IOException;
import java.nio.ByteBuffer;
import java.util.Arrays;
import java.util.Vector;

import sensor_msgs.CompressedImage;
import sensor_msgs.Image;
import std_msgs.Header;

/**
 * @author Tal Regev
 */
public class CvImage
{
   static protected final String TAG = "cv_bridge::CvImage";
   public Header header;
   public Mat image = new Mat();
   public String encoding = "";

   protected CvImage()
   {
   }

   public CvImage(final Header header, final String encoding)
   {
      this.header = header;
      this.encoding = encoding.toUpperCase();
      this.image = new Mat();
   }

   public CvImage(final Header header, final String encoding, final Mat image)
   {
      this.header = header;
      this.encoding = encoding.toUpperCase();
      this.image = image;
   }

   public final Image toImageMessage(final Image rosImage) throws IOException
   {
      rosImage.setHeader(header);
      rosImage.setEncoding(encoding.toLowerCase());

      rosImage.setWidth(image.cols());
      rosImage.setHeight(image.rows());
      rosImage.setStep((int) image.arrayStep());
      //// TODO: Handle the indian if needed;
      //// rosImage.setIsBigendian();

      ChannelBufferOutputStream stream = new ChannelBufferOutputStream(MessageBuffers.dynamicBuffer());
      byte[] imageInBytes = new byte[(int) image.arraySize()];
      ((ByteBuffer) image.createBuffer()).get(imageInBytes);
      stream.write(imageInBytes);

      //noinspection UnusedAssignment
      imageInBytes = null;

      rosImage.setData(stream.buffer());
      return rosImage;
   }

   //TODO add a compression parameter.
   public final CompressedImage toCompressedImageMessage(final CompressedImage rosImage, Format destinationFormat) throws Exception
   {
      rosImage.setHeader(header);
      Mat image;
      if (!encoding.equals(ImageEncodings.BGR8))
      {
         CvImage temp = CvImage.cvtColor(this, ImageEncodings.BGR8);
         image = temp.image;
      }
      else
      {
         image = this.image;
      }

      //from https://github.com/bytedeco/javacpp-presets/issues/29#issuecomment-6408082977
      BytePointer buffer = new BytePointer();

      //from http://docs.opencv.org/modules/highgui/doc/reading_and_writing_images_and_video.html#Mat imread(const string& filename, int flags)

      rosImage.setFormat(Format.valueOf(destinationFormat));
      opencv_imgcodecs.imencode(Format.getExtension(destinationFormat), image, buffer);

      ChannelBufferOutputStream stream = new ChannelBufferOutputStream(MessageBuffers.dynamicBuffer());
      //from https://github.com/bytedeco/javacpp-presets/issues/29#issuecomment-6408082977
      byte[] outputBuffer = new byte[(int) buffer.capacity()];
      buffer.get(outputBuffer);
      stream.write(outputBuffer);

      rosImage.setData(stream.buffer());
      return rosImage;
   }

   static public CvImage toCvCopy(final Image source) throws Exception
   {
      return CvImage.toCvCopyImpl(matFromImage(source), source.getHeader(), source.getEncoding(), "");
   }

   static public CvImage toCvCopy(final Image source, final String dst_encoding) throws Exception
   {
      return CvImage.toCvCopyImpl(matFromImage(source), source.getHeader(), source.getEncoding(), dst_encoding);
   }

   static public CvImage toCvCopy(final CompressedImage source) throws Exception
   {
      return CvImage.toCvCopyImpl(matFromImage(source), source.getHeader(), ImageEncodings.BGR8, "");
   }

   static public CvImage toCvCopy(final CompressedImage source, final String dst_encoding) throws Exception
   {
      return CvImage.toCvCopyImpl(matFromImage(source), source.getHeader(), ImageEncodings.BGR8, dst_encoding);
   }

   static public CvImage cvtColor(final CvImage source, String encoding) throws Exception
   {
      return toCvCopyImpl(source.image, source.header, source.encoding, encoding);
   }

   static protected CvImage toCvCopyImpl(final Mat source, final Header sourceHeader, final String sourceEncoding, final String destinationEncoding) throws Exception
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

   static protected Mat matFromImage(final Image source) throws Exception
   {
      byte[] imageInBytes = source.getData().array();
      imageInBytes = Arrays.copyOfRange(imageInBytes, source.getData().arrayOffset(), imageInBytes.length);
      String encoding = source.getEncoding().toUpperCase();
      Mat cvImage = new Mat(source.getHeight(), source.getWidth(), ImEncoding.getCvType(encoding));

      BytePointer bytePointer = new BytePointer(imageInBytes);
      cvImage = cvImage.data(bytePointer);
      return cvImage;
   }

   static protected Mat matFromImage(final CompressedImage source) throws Exception
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
