package us.ihmc.ihmcPerception.depthData;

import java.util.List;

public interface CollisionBoxProvider
{
   public List<CollisionDescription> getCollisionMesh(String jointName);
}
