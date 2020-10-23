package us.ihmc.robotics.physics;

import gnu.trove.map.hash.TObjectLongHashMap;

/**
 * Helper class that can be used to simplify the generation of collision masks and groups used with
 * {@link Collidable}s.
 * 
 * @author Sylvain Bertrand
 */
public class CollidableHelper
{
   private static final long EMPTY_VALUE = -1;
   private long nextCollisionMask = 0b1;
   private final TObjectLongHashMap<String> namedCollisionMask = new TObjectLongHashMap<>(64, 1.0f, EMPTY_VALUE);

   /**
    * A single instance of this helper should be used when creating multiple collidables for a robot
    * for instance.
    */
   public CollidableHelper()
   {
   }

   /**
    * Creates or retrieves the collision mask that is associated with the given name.
    * <p>
    * The collision mask can be used as the identifier of a collidable.
    * </p>
    * <p>
    * Only 32 collision masks can be created, check {@link #canAddCollisionMask()} to verify at any
    * time whether a new collision mask can be created.
    * </p>
    * 
    * @param name usually the name of the rigid-body the collision mask is for.
    * @return the value of the collision mask.
    */
   public long getCollisionMask(String name)
   {
      long collisionMask = namedCollisionMask.get(name);
      if (collisionMask == EMPTY_VALUE)
         return nextCollisionMask(name);
      else
         return collisionMask;
   }

   private long nextCollisionMask(String name)
   {
      if (!canAddCollisionMask())
         throw new RuntimeException("Max capacity reached.");

      long collisionMask = nextCollisionMask;
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
    * <p>
    * Create a {@link Collidable} with a group made of the other collidables' masks it should collide
    * with. Any collidable with a mask that is not in this group will not collide with the new
    * collidable.
    * </p>
    * 
    * @param collidables the names of collidables the group should represent.
    * @return the value of the collision group.
    */
   public long createCollisionGroup(String... collidables)
   {
      long group = 0b0;

      for (String collidable : collidables)
      {
         group |= getCollisionMask(collidable);
      }
      return group;
   }

   private static long shiftBitLeft(long value)
   {
      return value << 1;
   }
}
