package us.ihmc.avatar.networkProcessor.kinematicsToolboxModule.collision;

import gnu.trove.map.hash.TObjectIntHashMap;

/**
 * Helper class that can be used to simplify the generation of collision masks and groups used with
 * {@link KinematicsCollidable}s.
 * 
 * @author Sylvain Bertrand
 */
public class KinematicsCollidableHelper
{
   private static final int EMPTY_VALUE = -1;
   private int nextCollisionMask = 0b1;
   private final TObjectIntHashMap<String> namedCollisionMask = new TObjectIntHashMap<>(32, 1.0f, EMPTY_VALUE);

   /**
    * A single instance of this helper should be used when creating multiple collidables for a robot
    * for instance.
    */
   public KinematicsCollidableHelper()
   {
   }

   /**
    * Creates or retrieves the collision mask that is associated with the given name.
    * <p>
    * Only 32 collision masks can be created, check {@link #canAddCollisionMask()} to verify at any
    * time whether a new collision mask can be created.
    * </p>
    * 
    * @param name usually the name of the rigid-body the collision mask is for.
    * @return the value of the collision mask.
    */
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

   /**
    * Tests if another collision mask can be generated.
    * <p>
    * Only 32 collision masks can be created.
    * </p>
    * 
    * @return {@code true} if another collision mask can be created.
    */
   public boolean canAddCollisionMask()
   {
      return nextCollisionMask != 0;
   }

   /**
    * Computes the collision group from the names of collidables.
    * 
    * @param collidables the names of collidables the group should represent.
    * @return the value of the collision group.
    */
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
