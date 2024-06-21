package us.ihmc.perception.detections;

import perception_msgs.msg.dds.InstantDetectionMessage;
import perception_msgs.msg.dds.PersistentDetectionMessage;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.euclid.tuple3D.Point3D32;
import us.ihmc.perception.detections.YOLOv8.YOLOv8InstantDetection;
import us.ihmc.perception.detections.centerPose.CenterPoseInstantDetection;

import java.util.Iterator;

public class DetectionTools
{
   public static void toMessage(InstantDetection instantDetection, InstantDetectionMessage message)
   {
      message.setDetectedObjectClass(instantDetection.getDetectedObjectClass());
      message.setDetectedObjectName(instantDetection.getDetectedObjectName());
      message.setConfidence(instantDetection.getConfidence());
      message.getObjectPose().set(instantDetection.getPose());
      MessageTools.toMessage(instantDetection.getDetectionTime(), message.getDetectionTime());
   }

   public static void toMessage(YOLOv8InstantDetection yoloV8InstantDetection, InstantDetectionMessage message)
   {
      toMessage((InstantDetection) yoloV8InstantDetection, message);
      message.getYoloObjectPointCloud().clear();
      for (int i = 0; i < message.getYoloObjectPointCloud().capacity() && i < yoloV8InstantDetection.getObjectPointCloud().size(); ++i)
      {
         Point3D32 point = message.getYoloObjectPointCloud().add();
         point.set(yoloV8InstantDetection.getObjectPointCloud().get(i));
      }
   }

   public static void toMessage(CenterPoseInstantDetection centerPoseInstantDetection, InstantDetectionMessage message)
   {
      toMessage((InstantDetection) centerPoseInstantDetection, message);
      for (int i = 0; i < 8; ++i)
      {
         message.getCenterPoseBoundingBoxVertices()[i].set(centerPoseInstantDetection.getBoundingBoxVertices()[i]);
         message.getCenterPoseBoundingBox2dVertices()[i].set(centerPoseInstantDetection.getBoundingBoxVertices2D()[i]);
      }
   }

   public static <T extends InstantDetection> void toMessage(PersistentDetection<T> persistentDetection, PersistentDetectionMessage message)
   {
      message.getDetectionHistory().clear();
      Iterator<T> historyIterator = persistentDetection.getDetectionHistory().iterator();
      for (int i = 0; historyIterator.hasNext() && i < message.getDetectionHistory().capacity(); ++i)
         toMessage(historyIterator.next(), message.getDetectionHistory().add()); // TODO: Test which toMessage method this calls
   }
}
