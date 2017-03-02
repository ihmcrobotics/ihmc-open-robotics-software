package us.ihmc.robotics.stateMachines.conditionBasedStateMachine;

public abstract class SimpleState<E extends Enum<E>> extends State<E>
{
   public SimpleState(E stateEnum, E nextStateEnum)
   {
      super(stateEnum);

      if (nextStateEnum != null)
      {
         this.setDefaultNextState(nextStateEnum);
      }
   }

   public SimpleState(E stateEnum)
   {
      this(stateEnum, null);
   }

   @Override
   public void doTransitionIntoAction()
   {
   }

   @Override
   public void doTransitionOutOfAction()
   {
   }
}
