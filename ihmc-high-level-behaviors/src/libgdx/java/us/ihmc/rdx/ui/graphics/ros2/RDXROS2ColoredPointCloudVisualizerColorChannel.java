package us.ihmc.rdx.ui.graphics.ros2;

import org.bytedeco.opencl.global.OpenCL;
import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.global.opencv_imgcodecs;
import org.bytedeco.opencv.global.opencv_imgproc;
import org.bytedeco.opencv.opencv_core.Mat;
import perception_msgs.msg.dds.ImageMessage;
import us.ihmc.perception.BytedecoImage;
import us.ihmc.perception.OpenCLManager;
import us.ihmc.ros2.ROS2Topic;

public class RDXROS2ColoredPointCloudVisualizerColorChannel extends RDXROS2ColoredPointCloudVisualizerChannel
{
   private Mat yuv1420Image;
   private BytedecoImage color8UC3Image;
   private BytedecoImage color8UC4Image;

   public RDXROS2ColoredPointCloudVisualizerColorChannel(ROS2Topic<ImageMessage> topic)
   {
      super("Color ", topic);
   }

   @Override
   protected void initialize(OpenCLManager openCLManager)
   {
      yuv1420Image = new Mat(1, 1, opencv_core.CV_8UC1);
      color8UC3Image = new BytedecoImage(imageWidth, imageHeight, opencv_core.CV_8UC3);
      color8UC4Image = new BytedecoImage(imageWidth, imageHeight, opencv_core.CV_8UC4);
      color8UC4Image.createOpenCLImage(openCLManager, OpenCL.CL_MEM_READ_ONLY);
   }

   public void update(OpenCLManager openCLManager)
   {
      super.update(openCLManager);

      opencv_imgcodecs.imdecode(decompressionInput.getInputMat(), opencv_imgcodecs.IMREAD_UNCHANGED, yuv1420Image);

      opencv_imgproc.cvtColor(yuv1420Image, color8UC3Image.getBytedecoOpenCVMat(), opencv_imgproc.COLOR_YUV2RGBA_I420);
      // Put the depth image into OpenCL buffer
      opencv_imgproc.cvtColor(color8UC3Image.getBytedecoOpenCVMat(), color8UC4Image.getBytedecoOpenCVMat(), opencv_imgproc.COLOR_RGB2RGBA);
   }

   public BytedecoImage getColor8UC4Image()
   {
      return color8UC4Image;
   }
}
