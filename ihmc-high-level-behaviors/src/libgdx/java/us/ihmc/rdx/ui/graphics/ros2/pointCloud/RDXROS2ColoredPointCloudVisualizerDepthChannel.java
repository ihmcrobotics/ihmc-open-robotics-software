package us.ihmc.rdx.ui.graphics.ros2.pointCloud;

import org.bytedeco.opencl.global.OpenCL;
import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.opencv_core.Mat;
import perception_msgs.msg.dds.ImageMessage;
import us.ihmc.perception.BytedecoImage;
import us.ihmc.perception.opencl.OpenCLManager;
import us.ihmc.ros2.ROS2Topic;
import us.ihmc.tools.thread.SwapReference;

/**
 * Channel used for the depth part. Handles decompression.
 */
public class RDXROS2ColoredPointCloudVisualizerDepthChannel extends RDXROS2ColoredPointCloudVisualizerChannel
{
   private final Mat decompressedDepthImage = new Mat();
   private SwapReference<BytedecoImage> depth16UC1ImageSwapReference;

   public RDXROS2ColoredPointCloudVisualizerDepthChannel(ROS2Topic<ImageMessage> topic)
   {
      super("Depth ", topic);
   }

   @Override
   protected void initialize(OpenCLManager openCLManager)
   {
      depth16UC1ImageSwapReference = new SwapReference<>(() ->
      {
         BytedecoImage depth16UC1Image = new BytedecoImage(imageWidth, imageHeight, opencv_core.CV_16UC1);
         depth16UC1Image.createOpenCLImage(openCLManager, OpenCL.CL_MEM_READ_ONLY);
         getFrequencyText().ping();
         return depth16UC1Image;
      });
   }

   @Override
   protected void decompress()
   {
      synchronized (imageMessageSwapReference)
      {
         imageMessageDecoder.decodeMessage(imageMessageSwapReference.getForThreadTwo(), decompressedDepthImage);
         decompressedDepthImage.copyTo(depth16UC1ImageSwapReference.getForThreadOne().getBytedecoOpenCVMat());
      }
      depth16UC1ImageSwapReference.swap();
   }

   @Override
   protected Object getDecompressionAccessSyncObject()
   {
      return depth16UC1ImageSwapReference;
   }

   @Override
   public void destroy()
   {
      super.destroy();
      decompressedDepthImage.close();
   }

   public BytedecoImage getDepth16UC1Image()
   {
      return depth16UC1ImageSwapReference.getForThreadTwo();
   }
}
