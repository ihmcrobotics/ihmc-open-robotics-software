package us.ihmc.behaviors.behaviorTree.scene;

import behavior_msgs.msg.dds.PersistentDetectionStatusMessage;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.perception.detections.PersistentDetection;

import java.time.Instant;

public class PersistentDetectionMessageTool
{
   private final transient FramePose3D detectionPose = new FramePose3D();
   private final transient RigidBodyTransform detectionTransform = new RigidBodyTransform();

   public void toMessage(ROS2SyncedRobotModel syncedRobot, Instant now, PersistentDetection persistentDetection, PersistentDetectionStatusMessage message)
   {
      String last4Digits = String.format("%04d", Math.abs(persistentDetection.getID().getLeastSignificantBits() % 10000));
      message.setId(last4Digits);
      message.setObjectClass(persistentDetection.getDetectedObjectClass());
      message.setDecayingFrequency(persistentDetection.getDetectionFrequencyDecaying(now));
      message.setHistorySize(persistentDetection.getHistorySize());
      message.setIsStable(persistentDetection.isStable(now));

      detectionPose.setToZero(syncedRobot.getReferenceFrames().getExperimentalCameraFrame());
      detectionPose.set(persistentDetection.getFilteredTransformToCamera());
      detectionPose.changeFrame(ReferenceFrame.getWorldFrame());
      detectionTransform.set(detectionPose);
      MessageTools.toMessage(detectionTransform, message.getTransformToWorld());

      MessageTools.toMessage(persistentDetection.getFilteredTransformToCamera(), message.getTransformToCamera());
   }
}
