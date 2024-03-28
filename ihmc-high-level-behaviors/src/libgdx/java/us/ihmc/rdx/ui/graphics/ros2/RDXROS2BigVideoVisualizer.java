package us.ihmc.rdx.ui.graphics.ros2;

import imgui.internal.ImGui;
import imgui.type.ImBoolean;
import org.bytedeco.javacpp.BytePointer;
import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.global.opencv_imgcodecs;
import org.bytedeco.opencv.global.opencv_imgproc;
import org.bytedeco.opencv.opencv_core.Mat;
import perception_msgs.msg.dds.BigVideoPacket;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.idl.IDLSequence;
import us.ihmc.pubsub.DomainFactory.PubSubImplementation;
import us.ihmc.pubsub.common.SampleInfo;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.imgui.ImPlotDoublePlot;
import us.ihmc.rdx.ui.graphics.RDXMessageSizeReadout;
import us.ihmc.rdx.ui.graphics.RDXOpenCVVideoVisualizer;
import us.ihmc.robotics.time.TimeTools;
import us.ihmc.ros2.ROS2Topic;
import us.ihmc.ros2.RealtimeROS2Node;
import us.ihmc.tools.string.StringTools;

public class RDXROS2BigVideoVisualizer extends RDXOpenCVVideoVisualizer
{
   private final String titleBeforeAdditions;
   private final PubSubImplementation pubSubImplementation;
   private final ROS2Topic<BigVideoPacket> topic;
   private RealtimeROS2Node realtimeROS2Node = null;
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final ImBoolean subscribed = new ImBoolean(false);
   private final BigVideoPacket videoPacket = new BigVideoPacket();
   private final SampleInfo sampleInfo = new SampleInfo();
   private final Object syncObject = new Object();
   private final byte[] messageDataHeapArray = new byte[25000000];
   private final BytePointer messageEncodedBytePointer = new BytePointer(25000000);
   private final Mat inputJPEGMat = new Mat(1, 1, opencv_core.CV_8UC1);
   private final Mat inputYUVI420Mat = new Mat(1, 1, opencv_core.CV_8UC1);
   private final ImPlotDoublePlot delayPlot = new ImPlotDoublePlot("Delay", 30);
   private final RDXMessageSizeReadout messageSizeReadout = new RDXMessageSizeReadout();

   public RDXROS2BigVideoVisualizer(String title, PubSubImplementation pubSubImplementation, ROS2Topic<BigVideoPacket> topic)
   {
      super(title + " (ROS 2)", topic.getName(), false);
      titleBeforeAdditions = title;
      this.pubSubImplementation = pubSubImplementation;
      this.topic = topic;

      setActivenessChangeCallback(isAlive ->
      {
         if (isAlive && realtimeROS2Node == null)
         {
            subscribe();
         }
         else if (!isAlive && realtimeROS2Node != null)
         {
            unsubscribe();
         }
      });
   }

   private void subscribe()
   {
      subscribed.set(true);
      realtimeROS2Node = ROS2Tools.createRealtimeROS2Node(pubSubImplementation, StringTools.titleToSnakeCase(titleBeforeAdditions));
      // imdecode takes the longest by far out of all this stuff
      // synchronize with the update method
      // YUV I420 has 1.5 times the height of the image
      realtimeROS2Node.createSubscription(topic, subscriber ->
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
               int numberOfBytes = imageEncodedTByteArrayList.size();
               imageEncodedTByteArrayList.toArray(messageDataHeapArray);
               messageEncodedBytePointer.put(messageDataHeapArray, 0, numberOfBytes);
               messageEncodedBytePointer.limit(numberOfBytes);

               inputJPEGMat.cols(numberOfBytes);
               inputJPEGMat.data(messageEncodedBytePointer);

               messageSizeReadout.update(numberOfBytes);
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
      messageSizeReadout.renderImGuiWidgets();
      if (getHasReceivedOne())
      {
         getFrequencyPlot().renderImGuiWidgets();
         delayPlot.renderImGuiWidgets();
      }
   }

   @Override
   public void destroy()
   {
      unsubscribe();
      super.destroy();
   }

   private void unsubscribe()
   {
      subscribed.set(false);
      if (realtimeROS2Node != null)
      {
         realtimeROS2Node.destroy();
         realtimeROS2Node = null;
      }
   }

   public boolean isSubscribed()
   {
      return subscribed.get();
   }
}
