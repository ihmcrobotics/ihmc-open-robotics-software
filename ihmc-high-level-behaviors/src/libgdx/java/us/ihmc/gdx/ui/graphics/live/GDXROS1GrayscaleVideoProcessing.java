package us.ihmc.gdx.ui.graphics.live;

import org.bytedeco.javacpp.BytePointer;
import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.global.opencv_imgproc;
import org.bytedeco.opencv.opencv_core.Mat;
import org.jboss.netty.buffer.ChannelBuffer;
import us.ihmc.perception.BytedecoOpenCVTools;
import us.ihmc.perception.ROSOpenCVTools;

public class GDXROS1GrayscaleVideoProcessing implements GDXROS1VideoProcessor
{
   private Mat input8UC1Mat;
   private Mat input16UC1Mat;

   @Override
   public void prepare(int imageWidth, int imageHeight, ChannelBuffer nettyChannelBuffer)
   {
      if (input16UC1Mat == null || input16UC1Mat.rows() != imageHeight || input16UC1Mat.cols() != imageWidth)
      {
         input16UC1Mat = new Mat(imageHeight, imageWidth, opencv_core.CV_16UC1);
         input8UC1Mat = new Mat(imageHeight, imageWidth, opencv_core.CV_8UC1);
      }

      ROSOpenCVTools.backMatWithNettyBuffer(input16UC1Mat, nettyChannelBuffer);
      BytedecoOpenCVTools.clampTo8BitUnsignedChar(input16UC1Mat, input8UC1Mat, 0.0, 255.0);
   }

   @Override
   public void synchronizedPackPanelImage(Mat panelImageToPack, BytePointer rgba8888BytePointerToPack)
   {
      opencv_imgproc.cvtColor(input8UC1Mat, panelImageToPack, opencv_imgproc.COLOR_GRAY2RGBA);
   }
}
