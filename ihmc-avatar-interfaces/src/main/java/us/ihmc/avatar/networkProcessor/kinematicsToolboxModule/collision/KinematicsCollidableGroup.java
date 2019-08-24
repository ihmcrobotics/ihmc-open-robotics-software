package us.ihmc.avatar.networkProcessor.kinematicsToolboxModule.collision;

import java.util.ArrayList;
import java.util.Collection;
import java.util.List;
import java.util.stream.Collectors;

public class KinematicsCollidableGroup
{
   private final int groupId;
   private final List<KinematicsCollidable> collidables = new ArrayList<>();

   public KinematicsCollidableGroup(int groupId)
   {
      this.groupId = groupId;
   }

   public void addCollidable(KinematicsCollidable collidable)
   {
      collidables.add(collidable);
   }

   public void addCollidables(KinematicsCollidable... collidables)
   {
      for (KinematicsCollidable collidable : collidables)
         addCollidable(collidable);
   }

   public void addCollidables(Collection<? extends KinematicsCollidable> collidables)
   {
      for (KinematicsCollidable collidable : collidables)
         addCollidable(collidable);
   }

   public List<KinematicsCollisionResult> evaluateCollisions(KinematicsCollidable collidable)
   {
      return collidables.stream().map(collidable::evaluateCollision).collect(Collectors.toList());
   }

   public List<KinematicsCollisionResult> evaluateCollisions(KinematicsCollidableGroup other)
   {
      return collidables.stream().flatMap(collidable -> other.evaluateCollisions(collidable).stream()).collect(Collectors.toList());
   }

   public int getGroupId()
   {
      return groupId;
   }

   public List<KinematicsCollidable> getCollidables()
   {
      return collidables;
   }
}
