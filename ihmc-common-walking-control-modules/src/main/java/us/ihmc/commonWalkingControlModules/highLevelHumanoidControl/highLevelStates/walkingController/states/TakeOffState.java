package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.walkingController.states;

public class TakeOffState extends AbstractJumpingState
{
   private static final JumpStateEnum stateEnum = JumpStateEnum.TAKE_OFF;

   public TakeOffState()
   {
      super(stateEnum);
      // TODO Auto-generated constructor stub
   }

   @Override
   public boolean isDone()
   {
      // TODO Auto-generated method stub
      return false;
   }

   @Override
   public void doAction()
   {
      // TODO Auto-generated method stub

   }

   @Override
   public void doTransitionIntoAction()
   {
      // TODO Auto-generated method stub

   }

   @Override
   public void doTransitionOutOfAction()
   {
      // TODO Auto-generated method stub

   }
}
