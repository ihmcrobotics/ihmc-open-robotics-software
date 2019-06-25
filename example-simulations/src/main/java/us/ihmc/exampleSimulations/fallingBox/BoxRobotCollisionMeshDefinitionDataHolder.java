package us.ihmc.exampleSimulations.fallingBox;

import us.ihmc.euclid.shape.primitives.Box3D;
import us.ihmc.robotics.robotDescription.collisionMeshDefinitionData.BoxCollisionMeshDefinitionData;
import us.ihmc.robotics.robotDescription.collisionMeshDefinitionData.CollisionMeshDefinitionData;
import us.ihmc.robotics.robotDescription.collisionMeshDefinitionData.CollisionMeshDefinitionDataHolder;

public class BoxRobotCollisionMeshDefinitionDataHolder extends CollisionMeshDefinitionDataHolder
{
   public BoxRobotCollisionMeshDefinitionDataHolder(Box3D box)
   {
      CollisionMeshDefinitionData bodyCollisionMeshData = new BoxCollisionMeshDefinitionData("bodyJoint", box.getSizeX(), box.getSizeY(), box.getSizeZ());
      addCollisionMeshDefinitionData(bodyCollisionMeshData);
   }
}
