package us.ihmc.perception.detections.foundationPose;

import us.ihmc.communication.crdt.CRDTInfo;
import us.ihmc.communication.ros2.ROS2ActorDesignation;
import us.ihmc.communication.ros2.sync.ROS2PeerClockOffsetEstimator;
import us.ihmc.perception.detections.InstantDetection;
import us.ihmc.perception.detections.yolo.YOLOv8InstantDetection;

import java.util.Collection;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

public class CategoryLevelFoundationPoseCommunicatorMap
{
   private final CRDTInfo crdtInfo;
   private final Map<String, CategoryLevelFoundationPoseCommunicator> communicators = new LinkedHashMap<>();

   public CategoryLevelFoundationPoseCommunicatorMap(ROS2PeerClockOffsetEstimator peerClockEstimator)
   {
      crdtInfo = new CRDTInfo(ROS2ActorDesignation.ROBOT, peerClockEstimator);
   }

   public CategoryLevelFoundationPoseCommunicator getOrCreateCommunicator(CategoryLevelFoundationPoseTarget target)
   {
      return communicators.computeIfAbsent(target.key(), key -> new CategoryLevelFoundationPoseCommunicator(target, crdtInfo));
   }

   public Collection<CategoryLevelFoundationPoseCommunicator> getCommunicators()
   {
      return communicators.values();
   }

   public void updateCommunicators()
   {
      for (CategoryLevelFoundationPoseCommunicator communicator : communicators.values())
         communicator.update();
   }

   public void updateSelectedPoseEstimation(CategoryLevelFoundationPoseTarget selectedTarget, YOLOv8InstantDetection detection)
   {
      CategoryLevelFoundationPoseCommunicator communicator = getOrCreateCommunicator(selectedTarget);
      communicator.updatePoseEstimation(detection);
   }

   public void updateSelectedPoseEstimation(CategoryLevelFoundationPoseTarget selectedTarget, List<InstantDetection> detections)
   {
      CategoryLevelFoundationPoseCommunicator communicator = getOrCreateCommunicator(selectedTarget);
      communicator.updatePoseEstimation(detections);
   }

   public void resetAllExcept(CategoryLevelFoundationPoseTarget activeTarget)
   {
      for (CategoryLevelFoundationPoseCommunicator communicator : communicators.values())
      {
         if (!communicator.getTarget().key().equals(activeTarget.key()))
            communicator.resetTracking();
      }
   }

   public void closeCommunicators()
   {
      for (CategoryLevelFoundationPoseCommunicator communicator : communicators.values())
         communicator.close();

      communicators.clear();
   }
}