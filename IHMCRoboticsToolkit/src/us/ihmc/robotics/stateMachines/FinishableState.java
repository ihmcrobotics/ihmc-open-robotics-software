package us.ihmc.robotics.stateMachines;

public abstract class FinishableState<E extends Enum<E>> extends State<E>
{
   public FinishableState(E stateEnum)
   {
      super(stateEnum);
   }

   public abstract boolean isDone();
}
