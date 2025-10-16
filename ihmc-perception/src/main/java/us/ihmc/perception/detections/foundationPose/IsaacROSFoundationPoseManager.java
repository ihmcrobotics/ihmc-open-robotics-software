package us.ihmc.perception.detections.foundationPose;

import us.ihmc.communication.crdt.CRDTInfo;
import us.ihmc.perception.detections.InstantDetection;

import java.util.EnumMap;
import java.util.List;
import java.util.Map;

// TODO: Find a better name. It's not really managing the communicators.
public class IsaacROSFoundationPoseManager implements AutoCloseable
{
   private final Map<IsaacROSFoundationPoseObject, IsaacROSFoundationPoseCommunicator> communicators;

   public IsaacROSFoundationPoseManager(CRDTInfo crdtInfo)
   {
      communicators = new EnumMap<>(IsaacROSFoundationPoseObject.class);
      for (IsaacROSFoundationPoseObject object : IsaacROSFoundationPoseObject.values())
      {
         communicators.put(object, new IsaacROSFoundationPoseCommunicator(object, crdtInfo));
      }
   }

   public IsaacROSFoundationPoseCommunicator getCommunicator(IsaacROSFoundationPoseObject object)
   {
      return communicators.get(object);
   }

   public void update()
   {
      for (IsaacROSFoundationPoseCommunicator communicator : communicators.values())
         communicator.update();
   }

   public void updateDetections(List<InstantDetection> latestDetections)
   {
      for (IsaacROSFoundationPoseCommunicator communicator : communicators.values())
         communicator.updateDetections(latestDetections);
   }

   @Override
   public void close()
   {
      for (IsaacROSFoundationPoseCommunicator communicator : communicators.values())
         communicator.close();
   }
}
