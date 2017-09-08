package us.ihmc.robotics;

public interface TickAndUpdatable
{
   public abstract void tickAndUpdate();

   public abstract void tickAndUpdate(double timeToSetInSeconds);
}
