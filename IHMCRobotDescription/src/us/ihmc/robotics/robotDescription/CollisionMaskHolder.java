package us.ihmc.robotics.robotDescription;

public interface CollisionMaskHolder
{
   public abstract int getCollisionGroup();
   public abstract void setCollisionGroup(int collisionGroup);

   public abstract int getCollisionMask();
   public abstract void setCollisionMask(int collisionMask);
}
