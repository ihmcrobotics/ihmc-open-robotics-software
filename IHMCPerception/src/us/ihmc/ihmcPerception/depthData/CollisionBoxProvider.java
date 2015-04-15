package us.ihmc.ihmcPerception.depthData;

import java.util.List;

import us.ihmc.ihmcPerception.depthData.collisionShapes.CollisionShape;

public interface CollisionBoxProvider
{
   public List<CollisionShape> getCollisionMesh(String jointName);
}
