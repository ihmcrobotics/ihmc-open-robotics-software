package us.ihmc.rdx.ui.graphics.ros2.yolo;

import imgui.ImGui;
import org.bytedeco.opencv.opencv_core.Mat;
import perception_msgs.ImageMessage;
import perception_msgs.YOLOv8AnnotationInfoList;
import us.ihmc.communication.PerceptionAPI;
import us.ihmc.communication.ros2.ROS2Heartbeat;
import us.ihmc.communication.ros2.sync.ROS2PeerClockOffsetEstimator;
import us.ihmc.perception.detections.yolo.YOLOv8AnnotationInfo;
import us.ihmc.perception.imageMessage.PixelFormat;
import us.ihmc.rdx.imgui.RDXPanel;
import us.ihmc.rdx.ui.graphics.RDXImageVisualizer;
import us.ihmc.rdx.ui.graphics.ros2.RDXROS2ImageMessageVisualizer;
import us.ihmc.rdx.ui.graphics.ros2.RDXROS2MultiTopicVisualizer;
import us.ihmc.jros2.ROS2Node;
import us.ihmc.jros2.ROS2Subscription;
import us.ihmc.jros2.ROS2Topic;

import java.util.List;

public class RDXROS2YOLOv8Visualizer extends RDXROS2MultiTopicVisualizer
{
   private final ROS2Node ros2Node;
   private final ROS2Topic<ImageMessage> colorImageTopic;
   private final RDXROS2ImageMessageVisualizer imageMessageVisualizer;

   private final YOLOv8AnnotationInfoList latestAnnotationInfo;
   private final ROS2Subscription<YOLOv8AnnotationInfoList> annotationInfoSubscription;

   private final ROS2Heartbeat demandYOLO;

   private final RDXROS2YOLOv8Settings settings;

   public RDXROS2YOLOv8Visualizer(String title,
                                  ROS2Node ros2Node,
                                  ROS2PeerClockOffsetEstimator ros2ClockOffsetEstimator,
                                  ROS2Topic<ImageMessage> colorImageTopic)
   {
      this(title, ros2Node, ros2Node, ros2ClockOffsetEstimator, colorImageTopic);
   }

   public RDXROS2YOLOv8Visualizer(String title,
                                  ROS2Node ros2Node,
                                  ROS2Node intraProcessNode,
                                  ROS2PeerClockOffsetEstimator ros2ClockOffsetEstimator,
                                  ROS2Topic<ImageMessage> colorImageTopic)
   {
      super(title);

      this.ros2Node = ros2Node;
      this.colorImageTopic = colorImageTopic;
      imageMessageVisualizer = new RDXROS2ImageMessageVisualizer(title, intraProcessNode, colorImageTopic)
      {
         @Override
         protected void setImage(RDXImageVisualizer imageVisualizer, Mat image, PixelFormat pixelFormat)
         {
            RDXROS2YOLOv8Visualizer.this.getFrequency(colorImageTopic).ping();
            annotateImage(image);
            super.setImage(imageVisualizer, image, pixelFormat);
         }
      };

      latestAnnotationInfo = new YOLOv8AnnotationInfoList();
      annotationInfoSubscription = ros2Node.createSubscription(PerceptionAPI.YOLO_ANNOTATION_INFO, reader -> this.setLatestAnnotationInfo(reader.read()));

      demandYOLO = new ROS2Heartbeat(ros2Node, PerceptionAPI.REQUEST_YOLO);

      settings = new RDXROS2YOLOv8Settings(ros2Node, ros2ClockOffsetEstimator);
   }

   @Override
   public void update()
   {
      super.update();
      imageMessageVisualizer.update();
   }

   @Override
   public void updateHeartbeat()
   {
      super.updateHeartbeat();
      imageMessageVisualizer.setActive(isActive());
      demandYOLO.setAlive(isActive());
      settings.update();
   }

   @Override
   public void renderImGuiWidgets()
   {
      if (ImGui.button(labels.get("Enable All")))
         settings.enableAllModels();
      ImGui.sameLine();
      if (ImGui.button(labels.get("Disable All")))
         settings.disableAllModels();

      settings.renderSettings();
      imageMessageVisualizer.renderImGuiWidgets();
   }

   @Override
   public RDXPanel getPanel()
   {
      return imageMessageVisualizer.getPanel();
   }

   @Override
   public void destroy()
   {
      super.destroy();
      imageMessageVisualizer.destroy();
      ros2Node.destroySubscription(annotationInfoSubscription);
      settings.destroy();
      demandYOLO.destroy();
   }

   private void annotateImage(Mat image)
   {
      synchronized (latestAnnotationInfo)
      {
         YOLOv8AnnotationInfo[] annotationInfos = new YOLOv8AnnotationInfo[latestAnnotationInfo.getAnnotationInfos().size()];
         for (int i = 0; i < annotationInfos.length; ++i)
            annotationInfos[i] = YOLOv8AnnotationInfo.fromMessage(latestAnnotationInfo.getAnnotationInfos().get(i));
         for (YOLOv8AnnotationInfo annotationInfo : annotationInfos)
            annotationInfo.drawMask(image, image, true, 0.5);
         for (YOLOv8AnnotationInfo annotationRecord : annotationInfos)
            annotationRecord.drawBoundingBox(image, image);
         for (YOLOv8AnnotationInfo annotationRecord : annotationInfos)
            annotationRecord.drawText(image, image, true, false);
      }
   }

   private void setLatestAnnotationInfo(YOLOv8AnnotationInfoList annotationInfo)
   {
      getFrequency(PerceptionAPI.YOLO_ANNOTATION_INFO).ping();
      synchronized (latestAnnotationInfo)
      {
         latestAnnotationInfo.set(annotationInfo);
      }
   }

   @Override
   public List<ROS2Topic<?>> getTopics()
   {
      return List.of(PerceptionAPI.YOLO_ANNOTATION_INFO, colorImageTopic);
   }
}
