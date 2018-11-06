package us.ihmc.humanoidRobotics.communication.wholeBodyTrajectoryToolboxAPI;

import java.util.Map;

import us.ihmc.mecano.multiBodySystem.RigidBody;
import us.ihmc.sensorProcessing.frames.ReferenceFrameHashCodeResolver;

public interface WholeBodyTrajectoryToolboxAPI<M>
{
   void set(M message, Map<Long, RigidBody> rigidBodyNamedBasedHashMap, ReferenceFrameHashCodeResolver referenceFrameResolver);
}