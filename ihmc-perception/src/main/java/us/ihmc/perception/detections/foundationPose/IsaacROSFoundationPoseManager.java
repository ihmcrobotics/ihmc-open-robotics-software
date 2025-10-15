package us.ihmc.perception.detections.foundationPose;

import us.ihmc.perception.detections.InstantDetection;

import java.util.HashMap;
import java.util.List;
import java.util.Map;

public class IsaacROSFoundationPoseManager implements AutoCloseable
{
   private final Map<IsaacROSFoundationPoseObject, IsaacROSFoundationPoseCommunicator> communicators;
   private final Map<IsaacROSFoundationPoseObject, IsaacROSFoundationPoseCommunicator> enabledCommunicators;

   public IsaacROSFoundationPoseManager()
   {
      int trackableObjectCount = IsaacROSFoundationPoseObject.values().length;

      communicators = new HashMap<>();
      enabledCommunicators = new HashMap<>();
      for (IsaacROSFoundationPoseObject object : IsaacROSFoundationPoseObject.values())
      {
         communicators.put(object, new IsaacROSFoundationPoseCommunicator(object));
      }
   }

   public void enableTracking(IsaacROSFoundationPoseObject object)
   {
      synchronized (enabledCommunicators)
      {
         enabledCommunicators.put(object, communicators.get(object));
      }
   }

   public void updateDetections(List<InstantDetection> latestDetections)
   {
      synchronized (enabledCommunicators)
      {
         for (IsaacROSFoundationPoseCommunicator communicator : enabledCommunicators.values())
            communicator.updateDetections(latestDetections);
      }
   }

   @Override
   public void close()
   {
      for (IsaacROSFoundationPoseCommunicator communicator : communicators.values())
         communicator.close();
   }
}
