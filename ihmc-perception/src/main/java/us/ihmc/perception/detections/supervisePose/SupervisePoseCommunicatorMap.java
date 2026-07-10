package us.ihmc.perception.detections.supervisePose;

import us.ihmc.communication.crdt.CRDTInfo;
import us.ihmc.communication.ros2.ROS2ActorDesignation;
import us.ihmc.communication.ros2.sync.ROS2PeerClockOffsetEstimator;
import us.ihmc.perception.detections.InstantDetection;
import us.ihmc.perception.detections.yolo.YOLOv8InstantDetection;

import java.util.Collection;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

public class SupervisePoseCommunicatorMap
{
   private final CRDTInfo crdtInfo;
   private final Map<String, SupervisePoseCommunicator> communicators = new LinkedHashMap<>();

   public SupervisePoseCommunicatorMap(ROS2PeerClockOffsetEstimator peerClockEstimator)
   {
      crdtInfo = new CRDTInfo(ROS2ActorDesignation.ROBOT, peerClockEstimator);
   }

   public SupervisePoseCommunicator getOrCreateCommunicator(SupervisePoseTarget target)
   {
      return communicators.computeIfAbsent(target.key(), key -> new SupervisePoseCommunicator(target, crdtInfo));
   }

   public Collection<SupervisePoseCommunicator> getCommunicators()
   {
      return communicators.values();
   }

   public void updateCommunicators()
   {
      for (SupervisePoseCommunicator communicator : communicators.values())
         communicator.update();
   }

   public void updateSelectedPoseEstimation(SupervisePoseTarget selectedTarget, YOLOv8InstantDetection detection)
   {
      SupervisePoseCommunicator communicator = getOrCreateCommunicator(selectedTarget);
      communicator.updatePoseEstimation(detection);
   }

   public void updateSelectedPoseEstimation(SupervisePoseTarget selectedTarget, List<InstantDetection> detections)
   {
      SupervisePoseCommunicator communicator = getOrCreateCommunicator(selectedTarget);
      communicator.updatePoseEstimation(detections);
   }

   public void resetAllExcept(SupervisePoseTarget activeTarget)
   {
      for (SupervisePoseCommunicator communicator : communicators.values())
      {
         if (!communicator.getTarget().key().equals(activeTarget.key()))
            communicator.resetTracking();
      }
   }

   public void closeCommunicators()
   {
      for (SupervisePoseCommunicator communicator : communicators.values())
         communicator.close();

      communicators.clear();
   }
}