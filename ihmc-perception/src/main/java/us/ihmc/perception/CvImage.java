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

package cv_bridge;

import org.bytedeco.javacpp.BytePointer;
import org.bytedeco.javacpp.opencv_core;
import org.bytedeco.javacpp.opencv_core.Mat;
import org.bytedeco.javacpp.opencv_imgcodecs;
import org.bytedeco.javacpp.opencv_imgproc;
import org.jboss.netty.buffer.ChannelBuffer;
import org.jboss.netty.buffer.ChannelBufferOutputStream;
import org.ros.internal.message.MessageBuffers;

import java.io.IOException;
import java.nio.ByteBuffer;
import java.util.Arrays;
import java.util.Vector;

import sensor_msgs.CompressedImage;
import sensor_msgs.Image;
import sensor_msgs.ImageEncodings;
import std_msgs.Header;

/**
 * @author Tal Regev
 */
@SuppressWarnings({"WeakerAccess"})
public class CvImage
{
    static protected final String TAG = "cv_bridge::CvImage";
    public Header header;
    public Mat image = new Mat();
    public String encoding = "";

    protected CvImage(){}

    public CvImage(final Header header, final String encoding)
    {
        this.header = header;
        this.encoding = encoding.toUpperCase();
        this.image = new Mat();
    }

    @SuppressWarnings("unused")
    public CvImage(final Header header, final String encoding,
                   final Mat image)
    {
        this.header = header;
        this.encoding = encoding.toUpperCase();
        this.image = image;
    }

    @SuppressWarnings("unused")
    public final Image toImageMsg(final Image ros_image) throws IOException {
        ros_image.setHeader(header);
        ros_image.setEncoding(encoding.toLowerCase());

        ros_image.setWidth(image.cols());
        ros_image.setHeight(image.rows());
        ros_image.setStep(image.arrayStep());
        //// TODO: Handle the indian if needed;
        //// ros_image.setIsBigendian();

        ChannelBufferOutputStream stream = new ChannelBufferOutputStream(MessageBuffers.dynamicBuffer());
        byte[] imageInBytes = new byte[image.arraySize()];
        ((ByteBuffer)image.createBuffer()).get(imageInBytes);
        stream.write(imageInBytes);

        //noinspection UnusedAssignment
        imageInBytes = null;

        ros_image.setData(stream.buffer());
        return ros_image;
    }

    //TODO add a compression parameter.
    public final CompressedImage toCompressedImageMsg(final CompressedImage ros_image, Format dst_format) throws Exception {
        ros_image.setHeader(header);
        Mat image;
        if(!encoding.equals(ImageEncodings.BGR8))
        {
            CvImage temp = CvImage.cvtColor(this, ImageEncodings.BGR8);
            image      = temp.image;
        }
        else
        {
            image = this.image;
        }

        //from https://github.com/bytedeco/javacpp-presets/issues/29#issuecomment-6408082977
        BytePointer buf = new BytePointer();

        //from http://docs.opencv.org/modules/highgui/doc/reading_and_writing_images_and_video.html#Mat imread(const string& filename, int flags)

        ros_image.setFormat(Format.valueOf(dst_format));
        opencv_imgcodecs.imencode(Format.getExtension(dst_format), image, buf);


        ChannelBufferOutputStream stream = new ChannelBufferOutputStream(MessageBuffers.dynamicBuffer());
        //from https://github.com/bytedeco/javacpp-presets/issues/29#issuecomment-6408082977
        byte[] outputBuffer = new byte[(int)buf.capacity()];
        buf.get(outputBuffer);
        stream.write(outputBuffer);

        ros_image.setData(stream.buffer());
        return ros_image;
    }

    @SuppressWarnings("unused")
    static public CvImage toCvCopy(final Image source) throws Exception {
        return CvImage.toCvCopyImpl(matFromImage(source), source.getHeader(), source.getEncoding(), "");
    }

    @SuppressWarnings("unused")
    static public CvImage toCvCopy(final Image source, final String dst_encoding) throws Exception {
        return CvImage.toCvCopyImpl(matFromImage(source), source.getHeader(), source.getEncoding(), dst_encoding);
    }

    @SuppressWarnings("unused")
    static public CvImage toCvCopy(final CompressedImage source) throws Exception {
        return CvImage.toCvCopyImpl(matFromImage(source), source.getHeader(), ImageEncodings.BGR8, "");
    }

    static public CvImage toCvCopy(final CompressedImage source,final String dst_encoding) throws Exception {
        return CvImage.toCvCopyImpl(matFromImage(source), source.getHeader(), ImageEncodings.BGR8, dst_encoding);
    }

    @SuppressWarnings("unused")
    static public CvImage cvtColor(final CvImage source, String encoding) throws Exception {
        return toCvCopyImpl(source.image, source.header, source.encoding, encoding);
    }

    static protected CvImage toCvCopyImpl(final Mat source,
                            final Header src_header,
                            final String src_encoding,
                            final String dst_encoding) throws Exception
    {
        /// @todo Handle endianness - e.g. 16-bit dc1394 camera images are big-endian
        /// Languages such as Java manage this for you so that Java code can run on any platform and programmers do not have to manage byte ordering.
        /// from http://www.yolinux.com/TUTORIALS/Endian-Byte-Order.html
        /// need to check if it true in our case with this cameras.

        // Copy metadata
        CvImage cvImage = new CvImage();
        cvImage.header = src_header;

        // Copy to new buffer if same encoding requested
        if (dst_encoding.isEmpty() || dst_encoding.equals(src_encoding))
        {
            cvImage.encoding = src_encoding;
            source.copyTo(cvImage.image);
        }
        else
        {
            // Convert the source data to the desired encoding
            final Vector<Integer> conversion_codes = ImEncoding.getConversionCode(src_encoding, dst_encoding);
            Mat image1 = source;
            Mat image2 = new Mat();


            for(int i=0; i < conversion_codes.size(); ++i)
            {
                int conversion_code = conversion_codes.get(i);
                if (conversion_code == ImEncoding.SAME_FORMAT) {
                    //convert from Same number of channels, but different bit depth

                    //double alpha = 1.0;
                    int src_depth = ImageEncodings.bitDepth(src_encoding);
                    int dst_depth = ImageEncodings.bitDepth(dst_encoding);
                    // Do scaling between CV_8U [0,255] and CV_16U [0,65535] images.
                    //from http://www.rubydoc.info/github/ruby-opencv/ruby-opencv/OpenCV/CvMat
                    //from http://docs.opencv.org/modules/core/doc/basic_structures.html
                    //TODO: check which value default for beta is ok.
                    int beta = 0;
                    int image2_type = opencv_core.CV_MAKETYPE(opencv_core.CV_MAT_DEPTH(ImEncoding.getCvType(dst_encoding)), image1.channels());
                    if (src_depth == 8 && dst_depth == 16)
                        image1.convertTo(image2, image2_type, 65535. / 255.,beta);
                    else if (src_depth == 16 && dst_depth == 8)
                        image1.convertTo(image2, image2_type, 255. / 65535.,beta);
                    else
                        image1.convertTo(image2, image2_type);
                }
                else
                {
                    // Perform color conversion
                    opencv_imgproc.cvtColor(image1, image2, conversion_codes.get(0));
                }
                image1 = image2;
            }
            cvImage.image = image2;
            cvImage.encoding = dst_encoding;
        }
        return cvImage;
    }

    static protected Mat matFromImage(final Image source) throws Exception {
        byte[] imageInBytes = source.getData().array();
        imageInBytes = Arrays.copyOfRange(imageInBytes,source.getData().arrayOffset(),imageInBytes.length);
        String encoding = source.getEncoding().toUpperCase();
        Mat cvImage = new Mat(source.getHeight(),source.getWidth(), ImEncoding.getCvType(encoding));

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
