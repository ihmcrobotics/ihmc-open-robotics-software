package us.ihmc.robotics.partNames;

import java.util.List;

import org.apache.commons.lang3.tuple.ImmutablePair;

import us.ihmc.euclid.tuple3D.Vector3D;

public interface ContactPointDefinitionHolder
{
   public abstract List<ImmutablePair<String, Vector3D>> getJointNameGroundContactPointMap();
}

