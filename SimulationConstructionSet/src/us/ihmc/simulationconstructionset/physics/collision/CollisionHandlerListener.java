package us.ihmc.simulationconstructionset.physics.collision;

import us.ihmc.simulationconstructionset.ExternalForcePoint;
import us.ihmc.simulationconstructionset.ExternalTorque;
import us.ihmc.simulationconstructionset.physics.CollisionShape;

public interface CollisionHandlerListener
{
   public abstract void collision(CollisionShape shapeA, CollisionShape shapeB, ExternalForcePoint forceA, ExternalForcePoint forceB, ExternalTorque torqueA, ExternalTorque torqueB);
}
