package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.jumpingController.states;

import us.ihmc.commonWalkingControlModules.controlModules.flight.JumpMessageHandler;
import us.ihmc.commonWalkingControlModules.controlModules.flight.WholeBodyMotionPlanner;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.HumanoidHighLevelControllerManager;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;

public class LandingState extends AbstractJumpState
{
   private static final JumpStateEnum stateEnum = JumpStateEnum.LANDING;

   public LandingState(WholeBodyMotionPlanner motionPlanner, JumpMessageHandler messageHandler, HighLevelHumanoidControllerToolbox controllerToolbox)
   {
      super(stateEnum, motionPlanner, messageHandler, controllerToolbox);
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
   public void doStateSpecificTransitionIntoAction()
   {
      
   }

   @Override
   public void doTransitionOutOfAction()
   {
      // TODO Auto-generated method stub

   }

}
