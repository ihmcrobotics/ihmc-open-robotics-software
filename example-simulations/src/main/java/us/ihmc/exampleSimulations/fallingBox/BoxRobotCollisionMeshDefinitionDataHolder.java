package us.ihmc.exampleSimulations.fallingBox;

import us.ihmc.euclid.geometry.Box3D;
import us.ihmc.robotics.robotDescription.collisionMeshDefinitionData.BoxCollisionMeshDefinitionData;
import us.ihmc.robotics.robotDescription.collisionMeshDefinitionData.CollisionMeshDefinitionData;
import us.ihmc.robotics.robotDescription.collisionMeshDefinitionData.CollisionMeshDefinitionDataHolder;

public class BoxRobotCollisionMeshDefinitionDataHolder extends CollisionMeshDefinitionDataHolder
{
   public BoxRobotCollisionMeshDefinitionDataHolder(Box3D box)
   {
      CollisionMeshDefinitionData bodyCollisionMeshData = new BoxCollisionMeshDefinitionData("bodyJoint", box.getLength(), box.getWidth(), box.getHeight());
      addCollisionMeshDefinitionData(bodyCollisionMeshData);
   }
}
