package us.ihmc.gdx.ui.graphics.live;

import controller_msgs.msg.dds.VideoPacket;
import imgui.internal.ImGui;
import org.bytedeco.javacpp.BytePointer;
import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.global.opencv_imgcodecs;
import org.bytedeco.opencv.global.opencv_imgproc;
import org.bytedeco.opencv.opencv_core.Mat;
import us.ihmc.communication.IHMCROS2Callback;
import us.ihmc.perception.BytedecoOpenCVTools;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.ros2.ROS2QosProfile;
import us.ihmc.ros2.ROS2Topic;

public class GDXROS2VideoVisualizer extends GDXOpenCVVideoVisualizer
{
   private final ROS2Node ros2Node;
   private final ROS2Topic<VideoPacket> topic;
   private final ROS2VideoFormat format;
   private Mat inputJPEGYUVI420Mat;
   private Mat input16UC1Mat;
   private Mat input8UC1Mat;
   private Mat bgr8Mat;

   public GDXROS2VideoVisualizer(String title, ROS2Node ros2Node, ROS2Topic<VideoPacket> topic, ROS2VideoFormat format)
   {
      super(title + " (ROS 2)", topic.getName(), false);
      this.ros2Node = ros2Node;
      this.topic = topic;
      this.format = format;
      new IHMCROS2Callback<>(ros2Node, topic, ROS2QosProfile.BEST_EFFORT(), message -> doReceiveMessageOnThread(() -> acceptMessage(message)));
   }

   private void acceptMessage(VideoPacket videoPacket)
   {
      if (bgr8Mat == null)
         bgr8Mat = new Mat(1); // allocate any amount of data, it'll be resized later

      byte[] dataArray = videoPacket.getData().toArray();
      BytePointer dataBytePointer = new BytePointer(dataArray);
      if (format == ROS2VideoFormat.JPEGYUVI420)
      {
         if (inputJPEGYUVI420Mat == null)
            inputJPEGYUVI420Mat = new Mat(1, 1, opencv_core.CV_8UC1);

         inputJPEGYUVI420Mat.cols(videoPacket.getData().size());
         inputJPEGYUVI420Mat.data(dataBytePointer);

         // Converts image to 3 channel BGR color image.
         // This should handle JPEG encoded YUV I420 and output BGR
         opencv_imgcodecs.imdecode(inputJPEGYUVI420Mat, opencv_imgcodecs.IMREAD_COLOR, bgr8Mat);
      }
      else // if (format == ROS2VideoFormat.CV16UC1)
      {
         if (input16UC1Mat == null)
         {
            int imageHeight = videoPacket.getImageHeight();
            int imageWidth = videoPacket.getImageWidth();
            input16UC1Mat = new Mat(imageHeight, imageWidth, opencv_core.CV_16UC1);
            input8UC1Mat = new Mat(imageHeight, imageWidth, opencv_core.CV_8UC1);
         }

         input16UC1Mat.data(dataBytePointer);

         BytedecoOpenCVTools.clampTo8BitUnsignedChar(input16UC1Mat, input8UC1Mat, 0.0, 255.0);
      }

      synchronized (this)
      {
         int imageWidth;
         int imageHeight;
         if (format == ROS2VideoFormat.JPEGYUVI420)
         {
            imageWidth = bgr8Mat.cols();
            imageHeight = bgr8Mat.rows();
         }
         else
         {
            imageWidth = input16UC1Mat.cols();
            imageHeight = input16UC1Mat.rows();
         }

         updateImageDimensions(imageWidth, imageHeight);

         if (format == ROS2VideoFormat.JPEGYUVI420)
         {
            opencv_imgproc.cvtColor(bgr8Mat, getRGBA8Mat(), opencv_imgproc.COLOR_BGR2BGRA);
         }
         else
         {
            opencv_imgproc.cvtColor(input8UC1Mat, getRGBA8Mat(), opencv_imgproc.COLOR_GRAY2RGBA);
         }
      }
   }

   @Override
   public void renderImGuiWidgets()
   {
      super.renderImGuiWidgets();
      ImGui.text(topic.getName());
      if (getHasReceivedOne())
         getFrequencyPlot().renderImGuiWidgets();
   }
}
