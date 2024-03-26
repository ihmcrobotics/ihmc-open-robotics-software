package us.ihmc.rdx.ui.graphics.ros2;

import perception_msgs.msg.dds.VideoPacket;
import imgui.internal.ImGui;
import org.bytedeco.javacpp.BytePointer;
import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.global.opencv_imgcodecs;
import org.bytedeco.opencv.global.opencv_imgproc;
import org.bytedeco.opencv.opencv_core.Mat;
import us.ihmc.ros2.ROS2Callback;
import us.ihmc.idl.IDLSequence;
import us.ihmc.perception.opencv.OpenCVTools;
import us.ihmc.rdx.ui.graphics.RDXOpenCVVideoVisualizer;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.ros2.ROS2Topic;

public class RDXROS2VideoVisualizer extends RDXOpenCVVideoVisualizer
{
   private final ROS2Node ros2Node;
   private final ROS2Topic<VideoPacket> topic;
   private final ROS2VideoFormat format;

   private Mat input16UC1Mat;
   private Mat input8UC1Mat;
   private Mat bgr8Mat;
   private final byte[] messageDataHeapArray = new byte[25000000];
   private final BytePointer messageEncodedBytePointer = new BytePointer(25000000);
   private Mat inputJPEGMat;
   private Mat inputYUVI420Mat;

   public RDXROS2VideoVisualizer(String title, ROS2Node ros2Node, ROS2Topic<VideoPacket> topic, ROS2VideoFormat format)
   {
      super(title + " (ROS 2)", topic.getName(), false);
      this.ros2Node = ros2Node;
      this.topic = topic;
      this.format = format;
      new ROS2Callback<>(ros2Node, topic, message -> doReceiveMessageOnThread(() -> acceptMessage(message)));
   }

   private void acceptMessage(VideoPacket videoPacket)
   {
      byte[] dataArray = videoPacket.getData().toArray();
      BytePointer dataBytePointer = new BytePointer(dataArray);
      if (format == ROS2VideoFormat.JPEGYUVI420)
      {
         if (inputJPEGMat == null)
         {
            inputJPEGMat = new Mat(1, 1, opencv_core.CV_8UC1);
            inputYUVI420Mat = new Mat(1, 1, opencv_core.CV_8UC1);
         }

         IDLSequence.Byte imageEncodedTByteArrayList = videoPacket.getData();
         imageEncodedTByteArrayList.toArray(messageDataHeapArray);
         messageEncodedBytePointer.put(messageDataHeapArray, 0, imageEncodedTByteArrayList.size());
         messageEncodedBytePointer.limit(imageEncodedTByteArrayList.size());

         inputJPEGMat.cols(imageEncodedTByteArrayList.size());
         inputJPEGMat.data(messageEncodedBytePointer);

         // imdecode takes the longest by far out of all this stuff
         opencv_imgcodecs.imdecode(inputJPEGMat, opencv_imgcodecs.IMREAD_UNCHANGED, inputYUVI420Mat);
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

         OpenCVTools.clampTo8BitUnsignedChar(input16UC1Mat, input8UC1Mat, 0.0, 255.0);
      }

      synchronized (this)
      {
         int imageWidth;
         int imageHeight;
         if (format == ROS2VideoFormat.JPEGYUVI420)
         {
            // YUV I420 has 1.5 times the height of the image
            imageWidth = inputYUVI420Mat.cols();
            imageHeight = (int) (inputYUVI420Mat.rows() / 1.5f);
         }
         else
         {
            imageWidth = input16UC1Mat.cols();
            imageHeight = input16UC1Mat.rows();
         }

         updateImageDimensions(imageWidth, imageHeight);

         if (format == ROS2VideoFormat.JPEGYUVI420)
         {
            opencv_imgproc.cvtColor(inputYUVI420Mat, getRGBA8Mat(), opencv_imgproc.COLOR_YUV2RGBA_I420);
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
