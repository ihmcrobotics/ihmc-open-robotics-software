package us.ihmc.perception.depthData;

import java.util.List;

import us.ihmc.perception.depthData.collisionShapes.CollisionShape;

public interface CollisionBoxProvider
{
   public List<CollisionShape> getCollisionMesh(String jointName);
}
