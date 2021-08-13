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



import org.bytedeco.javacpp.opencv_core;
import org.bytedeco.javacpp.opencv_imgproc;

import java.util.AbstractMap;
import java.util.Arrays;
import java.util.HashMap;
import java.util.Map;
import java.util.Vector;

import sensor_msgs.ImageEncodings;

//from http://stackoverflow.com/questions/1195206/is-there-a-java-equivalent-or-methodology-for-the-typedef-keyword-in-c
@SuppressWarnings("WeakerAccess")
class Pair<T1,T2> extends AbstractMap.SimpleImmutableEntry<T1,T2> {
    public Pair(T1 key, T2 value) {
        super(key, value);
    }
}

//from http://stackoverflow.com/questions/11047756/getting-enum-associated-with-int-value
@SuppressWarnings({"Convert2Diamond", "WeakerAccess"})
public enum Encoding { INVALID(-1), GRAY(0), RGB(1), BGR(2), RGBA(3), BGRA(4), YUV422(5), BAYER_RGGB(6), BAYER_BGGR(7), BAYER_GBRG(8), BAYER_GRBG(9);
    protected int encodingNumber;

    private static Map<Integer, Encoding> map = new HashMap<Integer, Encoding>();
    static {
        for (Encoding encoding : Encoding.values()) {
            map.put(encoding.encodingNumber, encoding);
        }
    }

    Encoding(final int encodingNumber) { this.encodingNumber = encodingNumber; }

    public static Encoding valueOf(int encodingNumber) {
        return map.get(encodingNumber);
    }
}

@SuppressWarnings({"Convert2Diamond", "WeakerAccess"})
class ImEncoding
{
    protected static final int SAME_FORMAT = -1;

    static int getCvType(final String enc) throws Exception {

        String encoding = enc.toLowerCase();
        if (encoding.equals(ImageEncodings.BGR8))   return opencv_core.CV_8UC3;
        if (encoding.equals(ImageEncodings.MONO8))  return opencv_core.CV_8UC1;
        if (encoding.equals(ImageEncodings.RGB8))   return opencv_core.CV_8UC3;
        if (encoding.equals(ImageEncodings.MONO16)) return opencv_core.CV_16UC1;
        if (encoding.equals(ImageEncodings.BGR16))  return opencv_core.CV_16UC3;
        if (encoding.equals(ImageEncodings.RGB16))  return opencv_core.CV_16UC3;
        if (encoding.equals(ImageEncodings.BGRA8))  return opencv_core.CV_8UC4;
        if (encoding.equals(ImageEncodings.RGBA8))  return opencv_core.CV_8UC4;
        if (encoding.equals(ImageEncodings.BGRA16)) return opencv_core.CV_16UC4;
        if (encoding.equals(ImageEncodings.RGBA16)) return opencv_core.CV_16UC4;

        // For bayer, return one-channel
        if (encoding.equals(ImageEncodings.BAYER_RGGB8)) return opencv_core.CV_8UC1;
        if (encoding.equals(ImageEncodings.BAYER_BGGR8)) return opencv_core.CV_8UC1;
        if (encoding.equals(ImageEncodings.BAYER_GBRG8)) return opencv_core.CV_8UC1;
        if (encoding.equals(ImageEncodings.BAYER_GRBG8)) return opencv_core.CV_8UC1;
        if (encoding.equals(ImageEncodings.BAYER_RGGB16)) return opencv_core.CV_16UC1;
        if (encoding.equals(ImageEncodings.BAYER_BGGR16)) return opencv_core.CV_16UC1;
        if (encoding.equals(ImageEncodings.BAYER_GBRG16)) return opencv_core.CV_16UC1;
        if (encoding.equals(ImageEncodings.BAYER_GRBG16)) return opencv_core.CV_16UC1;

        // Miscellaneous
        if (encoding.equals(ImageEncodings.YUV422)) return opencv_core.CV_8UC2;

        encoding = encoding.toUpperCase();

        //macro code
        if (encoding.equals(ImageEncodings.TYPE_8UC1)) return opencv_core.CV_8UC1;
        if (encoding.equals(ImageEncodings.TYPE_8UC2)) return opencv_core.CV_8UC2;
        if (encoding.equals(ImageEncodings.TYPE_8UC3)) return opencv_core.CV_8UC3;
        if (encoding.equals(ImageEncodings.TYPE_8UC4)) return opencv_core.CV_8UC4;
        if (encoding.equals(ImageEncodings.TYPE_8SC1)) return opencv_core.CV_8SC1;
        if (encoding.equals(ImageEncodings.TYPE_8SC2)) return opencv_core.CV_8SC2;
        if (encoding.equals(ImageEncodings.TYPE_8SC3)) return opencv_core.CV_8SC3;
        if (encoding.equals(ImageEncodings.TYPE_8SC4)) return opencv_core.CV_8SC4;
        if (encoding.equals(ImageEncodings.TYPE_16UC1)) return opencv_core.CV_16UC1;
        if (encoding.equals(ImageEncodings.TYPE_16UC2)) return opencv_core.CV_16UC2;
        if (encoding.equals(ImageEncodings.TYPE_16UC3)) return opencv_core.CV_16UC3;
        if (encoding.equals(ImageEncodings.TYPE_16UC4)) return opencv_core.CV_16UC4;
        if (encoding.equals(ImageEncodings.TYPE_16SC1)) return opencv_core.CV_16SC1;
        if (encoding.equals(ImageEncodings.TYPE_16SC2)) return opencv_core.CV_16SC2;
        if (encoding.equals(ImageEncodings.TYPE_16SC3)) return opencv_core.CV_16SC3;
        if (encoding.equals(ImageEncodings.TYPE_16SC4)) return opencv_core.CV_16SC4;
        if (encoding.equals(ImageEncodings.TYPE_32SC1)) return opencv_core.CV_32SC1;
        if (encoding.equals(ImageEncodings.TYPE_32SC2)) return opencv_core.CV_32SC2;
        if (encoding.equals(ImageEncodings.TYPE_32SC3)) return opencv_core.CV_32SC3;
        if (encoding.equals(ImageEncodings.TYPE_32SC4)) return opencv_core.CV_32SC4;
        if (encoding.equals(ImageEncodings.TYPE_32FC1)) return opencv_core.CV_32FC1;
        if (encoding.equals(ImageEncodings.TYPE_32FC2)) return opencv_core.CV_32FC2;
        if (encoding.equals(ImageEncodings.TYPE_32FC3)) return opencv_core.CV_32FC3;
        if (encoding.equals(ImageEncodings.TYPE_32FC4)) return opencv_core.CV_32FC4;
        if (encoding.equals(ImageEncodings.TYPE_64FC1)) return opencv_core.CV_64FC1;
        if (encoding.equals(ImageEncodings.TYPE_64FC2)) return opencv_core.CV_64FC2;
        if (encoding.equals(ImageEncodings.TYPE_64FC3)) return opencv_core.CV_64FC3;
        if (encoding.equals(ImageEncodings.TYPE_64FC4)) return opencv_core.CV_64FC4;

        throw new Exception("Unrecognized image encoding [" + encoding + "]");
    }

    @SuppressWarnings("unused")
    protected static int safeLongToInt(long l) {
        if (l < Integer.MIN_VALUE || l > Integer.MAX_VALUE) {
            throw new IllegalArgumentException
                    (l + " cannot be cast to int without changing its value.");
        }
        return (int) l;
    }

    protected static Map<Pair<Encoding, Encoding>, Vector<Integer>> getConversionCodes() {
        Map<Pair<Encoding, Encoding>, Vector<Integer>> res = new HashMap<Pair<Encoding, Encoding>, Vector<Integer>>();

        for(int i=0; i<=5; ++i) {
            res.put(new Pair<Encoding, Encoding>(Encoding.valueOf(i), Encoding.valueOf(i)),
                    new Vector<Integer>(Arrays.asList(new Integer[]{SAME_FORMAT})));
        }

        res.put(new Pair<Encoding, Encoding>(Encoding.GRAY, Encoding.RGB),
                new Vector<Integer>(Arrays.asList(new Integer[]{opencv_imgproc.COLOR_GRAY2RGB})));
        res.put(new Pair<Encoding, Encoding>(Encoding.GRAY, Encoding.BGR),
                new Vector<Integer>(Arrays.asList(new Integer[]{opencv_imgproc.COLOR_GRAY2BGR})));
        res.put(new Pair<Encoding, Encoding>(Encoding.GRAY, Encoding.RGBA),
                new Vector<Integer>(Arrays.asList(new Integer[]{opencv_imgproc.COLOR_GRAY2RGBA})));
        res.put(new Pair<Encoding, Encoding>(Encoding.GRAY, Encoding.BGRA),
                new Vector<Integer>(Arrays.asList(new Integer[]{opencv_imgproc.COLOR_GRAY2BGRA})));

        res.put(new Pair<Encoding, Encoding>(Encoding.RGB, Encoding.GRAY),
                new Vector<Integer>(Arrays.asList(new Integer[]{opencv_imgproc.COLOR_RGB2GRAY})));
        res.put(new Pair<Encoding, Encoding>(Encoding.RGB, Encoding.BGR),
                new Vector<Integer>(Arrays.asList(new Integer[]{opencv_imgproc.COLOR_RGB2BGR})));
        res.put(new Pair<Encoding, Encoding>(Encoding.RGB, Encoding.RGBA),
                new Vector<Integer>(Arrays.asList(new Integer[]{opencv_imgproc.COLOR_RGB2RGBA})));
        res.put(new Pair<Encoding, Encoding>(Encoding.RGB, Encoding.BGRA),
                new Vector<Integer>(Arrays.asList(new Integer[]{opencv_imgproc.COLOR_RGB2BGRA})));

        res.put(new Pair<Encoding, Encoding>(Encoding.BGR, Encoding.GRAY),
                new Vector<Integer>(Arrays.asList(new Integer[]{opencv_imgproc.COLOR_BGR2GRAY})));
        res.put(new Pair<Encoding, Encoding>(Encoding.BGR, Encoding.RGB),
                new Vector<Integer>(Arrays.asList(new Integer[]{opencv_imgproc.COLOR_BGR2RGB})));
        res.put(new Pair<Encoding, Encoding>(Encoding.BGR, Encoding.RGBA),
                new Vector<Integer>(Arrays.asList(new Integer[]{opencv_imgproc.COLOR_BGR2RGBA})));
        res.put(new Pair<Encoding, Encoding>(Encoding.BGR, Encoding.BGRA),
                new Vector<Integer>(Arrays.asList(new Integer[]{opencv_imgproc.COLOR_BGR2BGRA})));

        res.put(new Pair<Encoding, Encoding>(Encoding.RGBA, Encoding.GRAY),
                new Vector<Integer>(Arrays.asList(new Integer[]{opencv_imgproc.COLOR_RGBA2GRAY})));
        res.put(new Pair<Encoding, Encoding>(Encoding.RGBA, Encoding.RGB),
                new Vector<Integer>(Arrays.asList(new Integer[]{opencv_imgproc.COLOR_RGBA2RGB})));
        res.put(new Pair<Encoding, Encoding>(Encoding.RGBA, Encoding.BGR),
                new Vector<Integer>(Arrays.asList(new Integer[]{opencv_imgproc.COLOR_RGBA2BGR})));
        res.put(new Pair<Encoding, Encoding>(Encoding.RGBA, Encoding.BGRA),
                new Vector<Integer>(Arrays.asList(new Integer[]{opencv_imgproc.COLOR_RGBA2BGRA})));

        res.put(new Pair<Encoding, Encoding>(Encoding.BGRA, Encoding.GRAY),
                new Vector<Integer>(Arrays.asList(new Integer[]{opencv_imgproc.COLOR_BGRA2GRAY})));
        res.put(new Pair<Encoding, Encoding>(Encoding.BGRA, Encoding.RGB),
                new Vector<Integer>(Arrays.asList(new Integer[]{opencv_imgproc.COLOR_BGRA2RGB})));
        res.put(new Pair<Encoding, Encoding>(Encoding.BGRA, Encoding.BGR),
                new Vector<Integer>(Arrays.asList(new Integer[]{opencv_imgproc.COLOR_BGRA2BGR})));
        res.put(new Pair<Encoding, Encoding>(Encoding.BGRA, Encoding.RGBA),
                new Vector<Integer>(Arrays.asList(new Integer[]{opencv_imgproc.COLOR_BGRA2RGBA})));

        res.put(new Pair<Encoding, Encoding>(Encoding.YUV422, Encoding.GRAY),
                new Vector<Integer>(Arrays.asList(new Integer[]{opencv_imgproc.COLOR_YUV2GRAY_UYVY})));
        res.put(new Pair<Encoding, Encoding>(Encoding.YUV422, Encoding.RGB),
                new Vector<Integer>(Arrays.asList(new Integer[]{opencv_imgproc.COLOR_YUV2RGB_UYVY})));
        res.put(new Pair<Encoding, Encoding>(Encoding.YUV422, Encoding.BGR),
                new Vector<Integer>(Arrays.asList(new Integer[]{opencv_imgproc.COLOR_YUV2BGR_UYVY})));
        res.put(new Pair<Encoding, Encoding>(Encoding.YUV422, Encoding.RGBA),
                new Vector<Integer>(Arrays.asList(new Integer[]{opencv_imgproc.COLOR_YUV2RGBA_UYVY})));
        res.put(new Pair<Encoding, Encoding>(Encoding.YUV422, Encoding.BGRA),
                new Vector<Integer>(Arrays.asList(new Integer[]{opencv_imgproc.COLOR_YUV2BGRA_UYVY})));

        // Deal with Bayer
        res.put(new Pair<Encoding, Encoding>(Encoding.BAYER_RGGB, Encoding.GRAY),
                new Vector<Integer>(Arrays.asList(new Integer[]{opencv_imgproc.COLOR_BayerBG2GRAY})));
        res.put(new Pair<Encoding, Encoding>(Encoding.BAYER_RGGB, Encoding.RGB),
                new Vector<Integer>(Arrays.asList(new Integer[]{opencv_imgproc.COLOR_BayerBG2RGB})));
        res.put(new Pair<Encoding, Encoding>(Encoding.BAYER_RGGB, Encoding.BGR),
                new Vector<Integer>(Arrays.asList(new Integer[]{opencv_imgproc.COLOR_BayerBG2BGR})));

        res.put(new Pair<Encoding, Encoding>(Encoding.BAYER_BGGR, Encoding.GRAY),
                new Vector<Integer>(Arrays.asList(new Integer[]{opencv_imgproc.COLOR_BayerRG2GRAY})));
        res.put(new Pair<Encoding, Encoding>(Encoding.BAYER_BGGR, Encoding.RGB),
                new Vector<Integer>(Arrays.asList(new Integer[]{opencv_imgproc.COLOR_BayerRG2RGB})));
        res.put(new Pair<Encoding, Encoding>(Encoding.BAYER_BGGR, Encoding.BGR),
                new Vector<Integer>(Arrays.asList(new Integer[]{opencv_imgproc.COLOR_BayerRG2BGR})));

        res.put(new Pair<Encoding, Encoding>(Encoding.BAYER_GBRG, Encoding.GRAY),
                new Vector<Integer>(Arrays.asList(new Integer[]{opencv_imgproc.COLOR_BayerGR2GRAY})));
        res.put(new Pair<Encoding, Encoding>(Encoding.BAYER_GBRG, Encoding.RGB),
                new Vector<Integer>(Arrays.asList(new Integer[]{opencv_imgproc.COLOR_BayerGR2RGB})));
        res.put(new Pair<Encoding, Encoding>(Encoding.BAYER_GBRG, Encoding.BGR),
                new Vector<Integer>(Arrays.asList(new Integer[]{opencv_imgproc.COLOR_BayerGR2BGR})));

        res.put(new Pair<Encoding, Encoding>(Encoding.BAYER_GRBG, Encoding.GRAY),
                new Vector<Integer>(Arrays.asList(new Integer[]{opencv_imgproc.COLOR_BayerGB2GRAY})));
        res.put(new Pair<Encoding, Encoding>(Encoding.BAYER_GRBG, Encoding.RGB),
                new Vector<Integer>(Arrays.asList(new Integer[]{opencv_imgproc.COLOR_BayerGB2RGB})));
        res.put(new Pair<Encoding, Encoding>(Encoding.BAYER_GRBG, Encoding.BGR),
                new Vector<Integer>(Arrays.asList(new Integer[]{opencv_imgproc.COLOR_BayerGB2BGR})));

        return res;
    }

    protected static Encoding getEncoding(final String encoding)
    {
        String lEncoding = encoding.toLowerCase();

        if (lEncoding.equals(ImageEncodings.MONO8)) return Encoding.GRAY;
        if (lEncoding.equals(ImageEncodings.BGR8)) return Encoding.BGR;
        if (lEncoding.equals(ImageEncodings.RGB8)) return Encoding.RGB;
        if (lEncoding.equals(ImageEncodings.BGRA8)) return Encoding.BGRA;
        if (lEncoding.equals(ImageEncodings.RGBA8)) return Encoding.RGBA;
        if (lEncoding.equals(ImageEncodings.YUV422)) return Encoding.YUV422;

        if (lEncoding.equals(ImageEncodings.BAYER_RGGB8)) return Encoding.BAYER_RGGB;
        if (lEncoding.equals(ImageEncodings.BAYER_BGGR8)) return Encoding.BAYER_BGGR;
        if (lEncoding.equals(ImageEncodings.BAYER_GBRG8)) return Encoding.BAYER_GBRG;
        if (lEncoding.equals(ImageEncodings.BAYER_GRBG8)) return Encoding.BAYER_GRBG;

        // We don't support conversions to/from other types
        return Encoding.INVALID;
    }

    protected static Vector<Integer> getConversionCode(String src_encoding, String dst_encoding) throws Exception {
        Encoding src_encode = getEncoding(src_encoding);
        Encoding dst_encode = getEncoding(dst_encoding);

        boolean is_src_color_format  = ImageEncodings.isColor(src_encoding) ||
                ImageEncodings.isMono(src_encoding) ||
                ImageEncodings.isBayer(src_encoding) ||
            (src_encoding.toLowerCase().equals(ImageEncodings.YUV422));


        boolean is_dst_color_format = ImageEncodings.isColor(dst_encoding) ||
                ImageEncodings.isMono(dst_encoding) ||
                ImageEncodings.isBayer(dst_encoding) ||
            (dst_encoding.toLowerCase().equals(ImageEncodings.YUV422));


        boolean is_num_channels_the_same = ImageEncodings.numChannels(src_encoding) == ImageEncodings.numChannels(dst_encoding);

        // If we have no color info in the source, we can only convert to the same format which
        // was resolved in the previous condition. Otherwise, fail
        if (!is_src_color_format) {
            if (is_dst_color_format)
                throw new Exception("[" + src_encoding + "] is not a color format. but [" + dst_encoding +
                        "] is. The conversion does not make sense");
            if (!is_num_channels_the_same)
                throw new Exception("[" + src_encoding + "] and [" + dst_encoding + "] do not have the same number of channel");
            return new Vector<Integer>(1, SAME_FORMAT);
        }

        // If we are converting from a color type to a non color type, we can only do so if we stick
        // to the number of channels
        if (!is_dst_color_format) {
            if (!is_num_channels_the_same)
                throw new Exception("[" + src_encoding + "] is a color format but [" + dst_encoding + "] " +
                        "is not so they must have the same OpenCV type, CV_8UC3, CV16UC1 ....");
            return new Vector<Integer>(1, SAME_FORMAT);
        }

        // If we are converting from a color type to another type, then everything is fine
        final Map<Pair<Encoding, Encoding>, Vector<Integer>> CONVERSION_CODES = getConversionCodes();

        Pair<Encoding, Encoding> key = new Pair<Encoding, Encoding>(src_encode, dst_encode);
        Vector<Integer> res = CONVERSION_CODES.get(key);

        if (res == null)
            throw new Exception("Unsupported conversion from [" + src_encoding +
                    "] to [" + dst_encoding + "]");


        // And deal with depth differences if the colors are different
        if (ImageEncodings.bitDepth(src_encoding) != ImageEncodings.bitDepth(dst_encoding) && (getEncoding(src_encoding) != getEncoding(dst_encoding)))
            res.add(SAME_FORMAT);

        return res;
    }
}