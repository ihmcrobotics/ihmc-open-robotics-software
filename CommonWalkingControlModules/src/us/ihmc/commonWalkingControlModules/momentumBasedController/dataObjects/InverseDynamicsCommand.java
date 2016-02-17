package us.ihmc.commonWalkingControlModules.momentumBasedController.dataObjects;

public abstract class InverseDynamicsCommand<T extends InverseDynamicsCommand<T>>
{
   public abstract void set(T other);
}
