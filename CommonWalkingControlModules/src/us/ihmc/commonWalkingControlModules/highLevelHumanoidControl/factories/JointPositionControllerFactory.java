package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories;

import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.HighLevelBehavior;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.ICPAndMomentumBasedController;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.JointPositionHighLevelController;
import us.ihmc.commonWalkingControlModules.momentumBasedController.MomentumBasedController;

public class JointPositionControllerFactory implements HighLevelBehaviorFactory
{
   private final boolean transitionRequested;

   public JointPositionControllerFactory( boolean transitionRequested)
   {
      this.transitionRequested = transitionRequested;
   }

   @Override
   public HighLevelBehavior createHighLevelBehavior(VariousWalkingProviders variousWalkingProviders, VariousWalkingManagers variousWalkingManagers,
         MomentumBasedController momentumBasedController, ICPAndMomentumBasedController icpAndMomentumBasedController)
   {
      return new JointPositionHighLevelController(momentumBasedController, icpAndMomentumBasedController, variousWalkingProviders);
   }

   @Override
   public boolean isTransitionToBehaviorRequested()
   {
      return transitionRequested;
   }
}