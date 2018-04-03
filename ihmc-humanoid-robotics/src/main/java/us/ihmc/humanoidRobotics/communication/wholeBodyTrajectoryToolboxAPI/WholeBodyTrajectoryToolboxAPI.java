package us.ihmc.humanoidRobotics.communication.wholeBodyTrajectoryToolboxAPI;

import java.util.Map;

import controller_msgs.msg.dds.WaypointBasedTrajectoryMessage;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.sensorProcessing.frames.ReferenceFrameHashCodeResolver;

public interface WholeBodyTrajectoryToolboxAPI<M>
{

   void set(M message, Map<Long, RigidBody> rigidBodyNamedBasedHashMap, ReferenceFrameHashCodeResolver referenceFrameResolver);

}