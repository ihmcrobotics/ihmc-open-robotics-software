package us.ihmc.avatar.networkProcessor.kinematicsToolboxModule.collision;

import gnu.trove.map.hash.TObjectIntHashMap;

public class KinematicsCollidableHelper
{
   private static final int EMPTY_VALUE = -1;
   private int nextCollisionMask = 0b1;
   private final TObjectIntHashMap<String> namedCollisionMask = new TObjectIntHashMap<>(32, 1.0f, EMPTY_VALUE);

   public KinematicsCollidableHelper()
   {
   }

   public int getCollisionMask(String name)
   {
      int collisionMask = namedCollisionMask.get(name);
      if (collisionMask == EMPTY_VALUE)
         return nextCollisionMask(name);
      else
         return collisionMask;
   }

   private int nextCollisionMask(String name)
   {
      if (!canAddCollisionMask())
         throw new RuntimeException("Max capacity reached.");

      int collisionMask = nextCollisionMask;
      namedCollisionMask.put(name, collisionMask);
      nextCollisionMask = shiftBitLeft(nextCollisionMask);
      return collisionMask;
   }

   public boolean canAddCollisionMask()
   {
      return nextCollisionMask != 0;
   }

   public int createCollisionGroup(String... collidables)
   {
      int group = 0b0;

      for (String collidable : collidables)
      {
         group |= getCollisionMask(collidable);
      }
      return group;
   }

   private static int shiftBitLeft(int value)
   {
      return value << 1;
   }
}
