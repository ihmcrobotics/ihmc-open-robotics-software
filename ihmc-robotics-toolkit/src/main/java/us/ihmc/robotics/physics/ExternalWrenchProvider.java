package us.ihmc.robotics.physics;

import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyReadOnly;
import us.ihmc.mecano.spatial.interfaces.FixedFrameSpatialVectorBasics;

import java.util.function.Function;

public interface ExternalWrenchProvider
{
   void applyExternalWrenches(RigidBodyReadOnly rootBody, Function<RigidBodyReadOnly, FixedFrameSpatialVectorBasics> externalWrenches);
}
