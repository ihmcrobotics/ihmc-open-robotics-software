package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories;

import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.HighLevelBehavior;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.HighLevelPositionController;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.ICPAndMomentumBasedController;
import us.ihmc.commonWalkingControlModules.momentumBasedController.MomentumBasedController;

public class HighLevelPositionControllerFactory implements HighLevelBehaviorFactory
{
   private final boolean transitionRequested;

   public HighLevelPositionControllerFactory(boolean transitionRequested)
   {
      this.transitionRequested = transitionRequested;
   }
   
   @Override
   public HighLevelBehavior createHighLevelBehavior(VariousWalkingProviders variousWalkingProviders, VariousWalkingManagers variousWalkingManagers,
         MomentumBasedController momentumBasedController, ICPAndMomentumBasedController icpAndMomentumBasedController)
   {
      return new HighLevelPositionController(momentumBasedController);
   }

   @Override
   public boolean isTransitionToBehaviorRequested()
   {
      return transitionRequested;
      }

}
