package us.ihmc.rdx.ui.graphics.ros2.yolo;

import org.bytedeco.opencv.opencv_core.Mat;
import perception_msgs.msg.dds.ImageMessage;
import perception_msgs.msg.dds.YOLOv8ResultAnnotationInfo;
import us.ihmc.communication.PerceptionAPI;
import us.ihmc.communication.ros2.ROS2Heartbeat;
import us.ihmc.communication.ros2.sync.ROS2PeerClockOffsetEstimator;
import us.ihmc.perception.detections.yolo.YOLOv8AnnotationRecord;
import us.ihmc.perception.imageMessage.PixelFormat;
import us.ihmc.rdx.ui.graphics.RDXImageVisualizer;
import us.ihmc.rdx.ui.graphics.ros2.RDXROS2ImageMessageVisualizer;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.ros2.ROS2Subscription;
import us.ihmc.ros2.ROS2Topic;

public class RDXROS2YOLOv8Visualizer extends RDXROS2ImageMessageVisualizer
{
   private final YOLOv8ResultAnnotationInfo latestAnnotationInfo;
   private final ROS2Subscription<YOLOv8ResultAnnotationInfo> annotationInfoSubscription;

   private final ROS2Heartbeat demandYOLO;

   private final RDXROS2YOLOv8Settings settings;

   public RDXROS2YOLOv8Visualizer(String title,
                                  ROS2Node ros2Node,
                                  ROS2PeerClockOffsetEstimator ros2ClockOffsetEstimator,
                                  ROS2Topic<ImageMessage> colorImageTopic)
   {
      super(title, ros2Node, colorImageTopic);

      latestAnnotationInfo = new YOLOv8ResultAnnotationInfo();
      annotationInfoSubscription = ros2Node.createSubscription2(PerceptionAPI.YOLO_ANNOTATION_INFO, this::setLatestAnnotationInfo);

      demandYOLO = new ROS2Heartbeat(ros2Node, PerceptionAPI.REQUEST_YOLO);

      settings = new RDXROS2YOLOv8Settings(ros2Node, ros2ClockOffsetEstimator);
   }

   @Override
   public void updateHeartbeat()
   {
      super.updateHeartbeat();
      demandYOLO.setAlive(isActive());
      settings.update();
   }

   @Override
   public void renderImGuiWidgets()
   {
      settings.renderSettings();
      super.renderImGuiWidgets();
   }

   @Override
   public void destroy()
   {
      super.destroy();
      annotationInfoSubscription.remove();
      settings.destroy();
      demandYOLO.destroy();
   }

   @Override
   protected void updateVisualizer(RDXImageVisualizer imageVisualizer, Mat image, PixelFormat pixelFormat)
   {
      synchronized (latestAnnotationInfo)
      {
         YOLOv8AnnotationRecord[] annotationRecords = new YOLOv8AnnotationRecord[latestAnnotationInfo.getAnnotationRecords().size()];
         for (int i = 0; i < annotationRecords.length; ++i)
            annotationRecords[i] = YOLOv8AnnotationRecord.fromMessage(latestAnnotationInfo.getAnnotationRecords().get(i));
         for (YOLOv8AnnotationRecord annotationRecord : annotationRecords)
            annotationRecord.drawMask(image, image, true, 0.5);
         for (YOLOv8AnnotationRecord annotationRecord : annotationRecords)
            annotationRecord.drawBoundingBox(image, image);
         for (YOLOv8AnnotationRecord annotationRecord : annotationRecords)
            annotationRecord.drawText(image, image, true, false);
      }

      super.updateVisualizer(imageVisualizer, image, pixelFormat);
   }

   private void setLatestAnnotationInfo(YOLOv8ResultAnnotationInfo annotationInfo)
   {
      synchronized (latestAnnotationInfo)
      {
         latestAnnotationInfo.set(annotationInfo);
      }
   }
}
