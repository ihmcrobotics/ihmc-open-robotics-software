package us.ihmc.simulationconstructionset.util;

public interface TickAndUpdatable
{
   public abstract void tickAndUpdate();

   public abstract void tickAndUpdate(double timeToSetInSeconds);
}
