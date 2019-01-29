package us.ihmc.humanoidRobotics.communication.kinematicsPlanningToolboxAPI;

import java.util.Map;

import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.sensorProcessing.frames.ReferenceFrameHashCodeResolver;

public interface KinematicsPlanningToolboxAPI<M>
{
   void set(M message, Map<Integer, RigidBodyBasics> rigidBodyNamedBasedHashMap, ReferenceFrameHashCodeResolver referenceFrameResolver);
}
