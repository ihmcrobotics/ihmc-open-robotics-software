package us.ihmc.gdx.ui.graphics.live;

import controller_msgs.msg.dds.BigVideoPacket;
import imgui.internal.ImGui;
import org.bytedeco.javacpp.BytePointer;
import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.global.opencv_imgcodecs;
import org.bytedeco.opencv.global.opencv_imgproc;
import org.bytedeco.opencv.opencv_core.Mat;
import us.ihmc.commons.Conversions;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.gdx.ui.tools.ImPlotDoublePlot;
import us.ihmc.idl.IDLSequence;
import us.ihmc.pubsub.DomainFactory.PubSubImplementation;
import us.ihmc.pubsub.common.SampleInfo;
import us.ihmc.robotics.time.TimeTools;
import us.ihmc.ros2.ROS2QosProfile;
import us.ihmc.ros2.ROS2Topic;
import us.ihmc.ros2.RealtimeROS2Node;
import us.ihmc.tools.string.StringTools;

import java.time.Instant;
import java.time.ZonedDateTime;
import java.time.temporal.ChronoUnit;
import java.time.temporal.TemporalField;

public class GDXROS2BigVideoVisualizer extends GDXOpenCVVideoVisualizer
{
   private final ROS2Topic<BigVideoPacket> topic;
   private final RealtimeROS2Node realtimeROS2Node;
   private final BigVideoPacket videoPacket = new BigVideoPacket();
   private final SampleInfo sampleInfo = new SampleInfo();
   private final Object syncObject = new Object();
   private final byte[] messageDataHeapArray = new byte[25000000];
   private final BytePointer messageEncodedBytePointer = new BytePointer(25000000);
   private final Mat inputJPEGMat = new Mat(1, 1, opencv_core.CV_8UC1);
   private final Mat inputYUVI420Mat = new Mat(1, 1, opencv_core.CV_8UC1);
   private final ImPlotDoublePlot delayPlot = new ImPlotDoublePlot("Delay", 30);

   public GDXROS2BigVideoVisualizer(String title, PubSubImplementation pubSubImplementation, ROS2Topic<BigVideoPacket> topic)
   {
      super(title + " (ROS 2)", topic.getName(), false);
      this.topic = topic;
      this.realtimeROS2Node = ROS2Tools.createRealtimeROS2Node(pubSubImplementation, StringTools.titleToSnakeCase(title));
      ROS2Tools.createCallbackSubscription(realtimeROS2Node, topic, ROS2QosProfile.BEST_EFFORT(), subscriber ->
      {
         synchronized (syncObject)
         {
            videoPacket.getData().resetQuick();
            subscriber.takeNextData(videoPacket, sampleInfo);
            delayPlot.addValue(TimeTools.calculateDelay(videoPacket.getAcquisitionTimeSecondsSinceEpoch(), videoPacket.getAcquisitionTimeAdditionalNanos()));
         }
         doReceiveMessageOnThread(() ->
         {
            synchronized (syncObject)
            {
               IDLSequence.Byte imageEncodedTByteArrayList = videoPacket.getData();
               imageEncodedTByteArrayList.toArray(messageDataHeapArray);
               messageEncodedBytePointer.put(messageDataHeapArray, 0, imageEncodedTByteArrayList.size());
               messageEncodedBytePointer.limit(imageEncodedTByteArrayList.size());

               inputJPEGMat.cols(imageEncodedTByteArrayList.size());
               inputJPEGMat.data(messageEncodedBytePointer);
            }

            // imdecode takes the longest by far out of all this stuff
            opencv_imgcodecs.imdecode(inputJPEGMat, opencv_imgcodecs.IMREAD_UNCHANGED, inputYUVI420Mat);

            synchronized (this) // synchronize with the update method
            {
               // YUV I420 has 1.5 times the height of the image
               updateImageDimensions(inputYUVI420Mat.cols(), (int) (inputYUVI420Mat.rows() / 1.5f));
               opencv_imgproc.cvtColor(inputYUVI420Mat, getRGBA8Mat(), opencv_imgproc.COLOR_YUV2RGBA_I420);
            }
         });
      });
      realtimeROS2Node.spin();
   }

   @Override
   public void renderImGuiWidgets()
   {
      super.renderImGuiWidgets();
      ImGui.text(topic.getName());
      if (getHasReceivedOne())
      {
         getFrequencyPlot().renderImGuiWidgets();
         delayPlot.renderImGuiWidgets();
      }
   }

   @Override
   public void destroy()
   {
      super.destroy();
      realtimeROS2Node.destroy();
   }
}
