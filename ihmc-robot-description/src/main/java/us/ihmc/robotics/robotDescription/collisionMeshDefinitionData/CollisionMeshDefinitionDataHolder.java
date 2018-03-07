package us.ihmc.robotics.robotDescription.collisionMeshDefinitionData;

import java.util.ArrayList;
import java.util.List;

public class CollisionMeshDefinitionDataHolder
{
   private final List<CollisionMeshDefinitionData> collisionMeshDefinitionDataList = new ArrayList<>();
   private boolean isVisible = false;

   public void addCollisionMeshDefinitionData(CollisionMeshDefinitionData collisionMeshData)
   {
      collisionMeshDefinitionDataList.add(collisionMeshData);
   }

   public List<CollisionMeshDefinitionData> getCollisionMeshDefinitionData()
   {
      return collisionMeshDefinitionDataList;
   }

   public void setVisible(boolean value)
   {
      isVisible = value;
   }

   public boolean isVisible()
   {
      return isVisible;
   }
}
