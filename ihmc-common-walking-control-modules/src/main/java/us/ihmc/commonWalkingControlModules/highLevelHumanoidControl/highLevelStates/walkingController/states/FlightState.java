package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.walkingController.states;

import us.ihmc.commonWalkingControlModules.controlModules.foot.FeetJumpManager;
import us.ihmc.commonWalkingControlModules.controlModules.foot.WholeBodyMomentumManager;

public class FlightState extends AbstractJumpingState
{
   private static final JumpStateEnum stateEnum = JumpStateEnum.FLIGHT;
   private final WholeBodyMomentumManager wholeBodyMomentumManager;
   private final FeetJumpManager feetJumpManager;
   
   public FlightState(WholeBodyMomentumManager wholeBodyMomentumManager, FeetJumpManager feetJumpManager)
   {
      super(stateEnum);
      this.wholeBodyMomentumManager = wholeBodyMomentumManager;
      this.feetJumpManager = feetJumpManager;
   }

   @Override
   public boolean isDone()
   {
      return false;
   }

   @Override
   public void doAction()
   {
      wholeBodyMomentumManager.update(stateEnum);
      wholeBodyMomentumManager.compute();
      feetJumpManager.compute();
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
