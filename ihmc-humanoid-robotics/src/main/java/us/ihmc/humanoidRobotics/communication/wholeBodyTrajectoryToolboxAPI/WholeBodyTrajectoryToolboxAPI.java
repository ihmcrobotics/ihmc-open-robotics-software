package us.ihmc.humanoidRobotics.communication.wholeBodyTrajectoryToolboxAPI;

import java.util.Map;

import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.sensorProcessing.frames.ReferenceFrameHashCodeResolver;

public interface WholeBodyTrajectoryToolboxAPI<M>
{
   void set(M message, Map<Integer, RigidBodyBasics> rigidBodyNamedBasedHashMap, ReferenceFrameHashCodeResolver referenceFrameResolver);
}