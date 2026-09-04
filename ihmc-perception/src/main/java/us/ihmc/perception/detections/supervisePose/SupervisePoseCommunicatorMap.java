package us.ihmc.perception.detections.supervisePose;

import us.ihmc.communication.crdt.CRDTInfo;
import us.ihmc.communication.ros2.ROS2ActorDesignation;
import us.ihmc.communication.ros2.sync.ROS2PeerClockOffsetEstimator;
import us.ihmc.perception.RawImage;
import us.ihmc.perception.detections.InstantDetection;
import us.ihmc.perception.detections.yolo.YOLOv8InstantDetection;
import us.ihmc.jros2.ROS2Node;

import java.util.Collection;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

public class SupervisePoseCommunicatorMap implements AutoCloseable
{
   private final CRDTInfo crdtInfo;

   private final Map<String, SupervisePoseCommunicator> communicators = new LinkedHashMap<>();

   /**
    * Publishes one image containing all active SupervisePose mesh overlays.
    */
   private final SupervisePoseMeshOverlayPublisher meshOverlayPublisher;

   public SupervisePoseCommunicatorMap(ROS2Node ros2Node, ROS2PeerClockOffsetEstimator peerClockEstimator)
   {
      crdtInfo = new CRDTInfo(ROS2ActorDesignation.ROBOT, peerClockEstimator);

      meshOverlayPublisher = new SupervisePoseMeshOverlayPublisher(ros2Node);
   }

   public SupervisePoseCommunicator getOrCreateCommunicator(SupervisePoseTarget target)
   {
      return communicators.computeIfAbsent(target.key(),
                                           key -> new SupervisePoseCommunicator(target, crdtInfo));
   }

   public Collection<SupervisePoseCommunicator> getCommunicators()
   {
      return communicators.values();
   }

   public void updateCommunicators()
   {
      for (SupervisePoseCommunicator communicator
            : communicators.values())
      {
         communicator.update();
      }
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

   /**
    * Draws every enabled and tracked communicator onto the same RGB image and
    * publishes the result on:
    *
    * /ihmc/supervisepose/overlayed_image
    *
    * Call this once per shared camera frame, not once per object.
    */

   public boolean hasTrackedCommunicator()
   {
      for (SupervisePoseCommunicator communicator : communicators.values())
      {
         if (communicator.isEnabled()
             && communicator.getState() == SupervisePoseCommunicator.State.TRACKING)
         {
            return true;
         }
      }

      return false;
   }

   public void publishMeshOverlay(RawImage colorImage)
   {
      meshOverlayPublisher.publish(colorImage, communicators.values());
   }

   public void resetAllExcept(SupervisePoseTarget activeTarget)
   {
      for (SupervisePoseCommunicator communicator
            : communicators.values())
      {
         if (!communicator.getTarget().key().equals(activeTarget.key()))
         {
            communicator.resetTracking(SupervisePoseCommunicator.ResetReason.UNKNOWN);
         }
      }
   }

   public void closeCommunicators()
   {
      for (SupervisePoseCommunicator communicator
            : communicators.values())
      {
         communicator.close();
      }

      communicators.clear();
   }

   @Override
   public void close()
   {
      closeCommunicators();
      meshOverlayPublisher.close();
   }
}