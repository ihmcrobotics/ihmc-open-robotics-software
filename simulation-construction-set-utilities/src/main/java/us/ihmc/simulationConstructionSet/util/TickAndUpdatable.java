package us.ihmc.simulationConstructionSet.util;

public interface TickAndUpdatable
{
   public abstract void tickAndUpdate();

   public abstract void tickAndUpdate(double timeToSetInSeconds);
}
