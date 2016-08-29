package us.ihmc.robotics.stateMachines;

import us.ihmc.robotics.math.trajectories.Finishable;

public abstract class FinishableState<E extends Enum<E>> extends State<E> implements Finishable
{

   public FinishableState(E stateEnum)
   {
      super(stateEnum);
   }
}
