package us.ihmc.commonWalkingControlModules.desiredFootStep.footstepGenerator;

import us.ihmc.euclid.tuple2D.interfaces.Vector2DReadOnly;

public interface DesiredVelocityProvider
{
   Vector2DReadOnly getDesiredVelocity();
}
