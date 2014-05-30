package us.ihmc.valkyrie.kinematics;

public interface ValkyrieJointInterface
{

   public abstract void setPosition(double q);

   public abstract void setVelocity(double qd);

   public abstract void setEffort(double effort);

   public abstract String getName();

   public abstract double getVelocity();

   public abstract double getEffort();

   public abstract double getPosition();

   public abstract double getDesiredEffort();

   public abstract void setDesiredEffort(double effort);

   public abstract double getDesiredPosition();

   public abstract void setDesiredPosition(double position);

   public abstract double getDesiredVelocity();

   public abstract void setDesiredVelocity(double velocity);

}