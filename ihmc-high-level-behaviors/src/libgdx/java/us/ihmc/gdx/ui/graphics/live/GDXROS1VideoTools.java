package us.ihmc.gdx.ui.graphics.live;

import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.global.opencv_imgproc;
import org.bytedeco.opencv.opencv_core.Mat;
import org.jboss.netty.buffer.ChannelBuffer;
import sensor_msgs.CompressedImage;
import sensor_msgs.Image;
import us.ihmc.perception.ImageEncodingTools;
import us.ihmc.perception.ROSOpenCVImage;
import us.ihmc.perception.ROSOpenCVTools;

import java.nio.Buffer;

public class GDXROS1VideoTools
{
//   private void decodeUsingOpenCV(Image image)
//   {
//      String encoding = image.getEncoding();
//      if (input16UC1Mat == null)
//      {
//         int cvType = ImageEncodingTools.getCvType(encoding);
//         input16UC1Mat = new Mat(image.getHeight(), image.getWidth(), cvType);
//         decodedImageMat = new Mat(image.getHeight(), image.getWidth(), opencv_core.CV_8UC4);
//         resizedImageMat = new Mat(image.getHeight(), image.getWidth(), opencv_core.CV_8UC4);
//      }
//
//      ChannelBuffer nettyChannelBuffer = image.getData();
//      ROSOpenCVTools.backMatWithNettyBuffer(input16UC1Mat, nettyChannelBuffer);
//
//      //      BytedecoOpenCVTools.clampTo8BitUnsignedChar(inputImageMat, input8UC1Mat, 0.0, 255.0);
//
//      if (ImageEncodingTools.isMono(encoding))
//      {
//         double min = 0.0;
//         double max = ImageEncodingTools.getMaxBitValue(encoding);
//         int depthType = -1; // output same type as input
//         opencv_core.normalize(input16UC1Mat, input16UC1Mat, min, max, opencv_core.NORM_MINMAX, depthType, opencv_core.noArray());
//      }
//
//      int conversionCode = ImageEncodingTools.getColorConversionCode(encoding, ImageEncodingTools.RGBA8).get(0);
//      opencv_imgproc.cvtColor(input16UC1Mat, decodedImageMat, conversionCode);
//
//      int sourceDepth = ImageEncodingTools.bitDepth(encoding);
//      if (sourceDepth == 16)
//      {
//         double alpha = 255.0 / 65535.0;
//         double beta = 0.0;
//         decodedImageMat.convertTo(resizedImageMat, opencv_core.CV_8UC4, alpha, beta);
//      }
//      else
//         decodedImageMat.convertTo(resizedImageMat, opencv_core.CV_8UC4); // TODO: Necessary?
//
//      //      pixmap.setPixels((ByteBuffer) resizedImageMat.createBuffer());
//   }

//   private void processIncomingMessageOnThread(CompressedImage compressedImage)
//   {
//      //      ChannelBuffer nettyChannelBuffer = compressedImage.getData();
//      //         decodedImageMat = new Mat(image.getHeight(), image.getWidth(), opencv_core.CV_8UC4);
//      //         resizedImageMat = new Mat(image.getHeight(), image.getWidth(), opencv_core.CV_8UC4);
//
//      try
//      {
//         ROSOpenCVImage ROSOpenCVImage = ROSOpenCVTools.toCvCopy(compressedImage, ImageEncodingTools.RGBA8);
//         Buffer buffer = ROSOpenCVImage.image.createBuffer();
//         //         ensureTextureReady(ROSOpenCVImage.image.cols(), ROSOpenCVImage.image.rows());
//         //         pixmap.setPixels((ByteBuffer) buffer);
//      }
//      catch (Exception e)
//      {
//         e.printStackTrace();
//      }
//   }
}
