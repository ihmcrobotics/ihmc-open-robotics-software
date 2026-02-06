package us.ihmc.perception.detections.foundationPose;

import us.ihmc.communication.crdt.CRDTInfo;
import us.ihmc.communication.ros2.ROS2ActorDesignation;
import us.ihmc.communication.ros2.sync.ROS2PeerClockOffsetEstimator;
import us.ihmc.perception.detections.InstantDetection;

import java.util.EnumMap;
import java.util.List;
import java.util.function.Consumer;

public class IsaacROSFoundationPoseCommunicatorMap extends EnumMap<IsaacROSFoundationPoseObject, IsaacROSFoundationPoseCommunicator>
{
   public IsaacROSFoundationPoseCommunicatorMap(ROS2PeerClockOffsetEstimator peerClockEstimator)
   {
      super(IsaacROSFoundationPoseObject.class);

      CRDTInfo crdtInfo = new CRDTInfo(ROS2ActorDesignation.ROBOT, peerClockEstimator);

      for (IsaacROSFoundationPoseObject object : IsaacROSFoundationPoseObject.values())
         put(object, new IsaacROSFoundationPoseCommunicator(object, crdtInfo));
   }

   /**
    * Calls the general update method for all communicators in this map.
    */
   public void updateCommunicators()
   {
      for (IsaacROSFoundationPoseCommunicator communicator : values())
         communicator.update();
   }

   /**
    * Updates pose estimation of all communicators in this map.
    * <p>
    * This method is designed to work in conjunction with
    * {@link us.ihmc.perception.detections.yolo.YOLOv8DetectionExecutor#addDetectionConsumerCallback(Consumer)}.
    *
    * @param latestDetections List of detections which may contain the object of interest.
    */
   public void updatePoseEstimations(List<InstantDetection> latestDetections)
   {
      for (IsaacROSFoundationPoseCommunicator communicator : values())
         communicator.updatePoseEstimation(latestDetections);
   }

   /**
    * Closes all communicators contained in this map.
    */
   public void closeCommunicators()
   {
      for (IsaacROSFoundationPoseCommunicator communicator : values())
         communicator.close();
   }
}
