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
import org.bytedeco.opencv.global.opencv_imgcodecs;
import org.bytedeco.opencv.opencv_core.*;
import org.jboss.netty.buffer.ChannelBufferOutputStream;
import org.ros.internal.message.MessageBuffers;

import java.io.IOException;
import java.nio.ByteBuffer;

import sensor_msgs.CompressedImage;
import sensor_msgs.Image;
import std_msgs.Header;

/**
 * Taken from https://github.com/talregev/android_cv_bridge/tree/master/cv_bridge_javacv/src/cv_bridge
 */
public class ROSOpenCVImage
{
   public Header header;
   public Mat image = new Mat();
   public String encoding = "";

   protected ROSOpenCVImage()
   {
   }

   public ROSOpenCVImage(final Header header, final String encoding)
   {
      this.header = header;
      this.encoding = encoding.toUpperCase();
      this.image = new Mat();
   }

   public ROSOpenCVImage(final Header header, final String encoding, final Mat image)
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
   public final CompressedImage toCompressedImageMessage(final CompressedImage rosImage, OpenCVImageFormat destinationFormat) throws Exception
   {
      rosImage.setHeader(header);
      Mat image;
      if (!encoding.equals(ImageEncodingTools.BGR8))
      {
         ROSOpenCVImage temp = ROSOpenCVTools.cvtColor(this, ImageEncodingTools.BGR8);
         image = temp.image;
      }
      else
      {
         image = this.image;
      }

      //from https://github.com/bytedeco/javacpp-presets/issues/29#issuecomment-6408082977
      BytePointer buffer = new BytePointer();

      //from http://docs.opencv.org/modules/highgui/doc/reading_and_writing_images_and_video.html#Mat imread(const string& filename, int flags)

      rosImage.setFormat(OpenCVImageFormat.valueOf(destinationFormat));
      opencv_imgcodecs.imencode(OpenCVImageFormat.getExtension(destinationFormat), image, buffer);

      ChannelBufferOutputStream stream = new ChannelBufferOutputStream(MessageBuffers.dynamicBuffer());
      //from https://github.com/bytedeco/javacpp-presets/issues/29#issuecomment-6408082977
      byte[] outputBuffer = new byte[(int) buffer.capacity()];
      buffer.get(outputBuffer);
      stream.write(outputBuffer);

      rosImage.setData(stream.buffer());
      return rosImage;
   }
}
