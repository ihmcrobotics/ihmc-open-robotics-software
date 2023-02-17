package us.ihmc.rdx.ui.graphics.ros2;

import org.bytedeco.opencl.global.OpenCL;
import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.global.opencv_imgcodecs;
import perception_msgs.msg.dds.ImageMessage;
import us.ihmc.perception.BytedecoImage;
import us.ihmc.perception.OpenCLManager;
import us.ihmc.ros2.ROS2Topic;

public class RDXROS2ColoredPointCloudVisualizerDepthChannel extends RDXROS2ColoredPointCloudVisualizerChannel
{
   private BytedecoImage depth16UC1Image;

   public RDXROS2ColoredPointCloudVisualizerDepthChannel(ROS2Topic<ImageMessage> topic)
   {
      super("Depth ", topic);
   }

   @Override
   protected void initialize(OpenCLManager openCLManager)
   {
      // Create OpenCV mat for depth image
      depth16UC1Image = new BytedecoImage(imageWidth, imageHeight, opencv_core.CV_16UC1);
      depth16UC1Image.createOpenCLImage(openCLManager, OpenCL.CL_MEM_READ_ONLY);
   }

   public void update(OpenCLManager openCLManager)
   {
      super.update(openCLManager);

      opencv_imgcodecs.imdecode(decompressionInput.getInputMat(), opencv_imgcodecs.IMREAD_UNCHANGED, depth16UC1Image.getBytedecoOpenCVMat());
   }

   public BytedecoImage getDepth16UC1Image()
   {
      return depth16UC1Image;
   }
}
