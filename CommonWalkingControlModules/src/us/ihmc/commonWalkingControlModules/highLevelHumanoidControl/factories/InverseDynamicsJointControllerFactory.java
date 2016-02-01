package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories;

import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.HighLevelBehavior;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.ICPAndMomentumBasedController;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.InverseDynamicsJointController;
import us.ihmc.commonWalkingControlModules.momentumBasedController.MomentumBasedController;

public class InverseDynamicsJointControllerFactory implements HighLevelBehaviorFactory
{
   private final boolean transitionRequested;

   public InverseDynamicsJointControllerFactory(boolean transitionRequested)
   {
      this.transitionRequested = transitionRequested;
   }

   @Override
   public HighLevelBehavior createHighLevelBehavior(VariousWalkingProviders variousWalkingProviders, VariousWalkingManagers variousWalkingManagers,
         MomentumBasedController momentumBasedController, ICPAndMomentumBasedController icpAndMomentumBasedController)
   {
      return new InverseDynamicsJointController(momentumBasedController, icpAndMomentumBasedController.getBipedSupportPolygons());
   }

   @Override
   public boolean isTransitionToBehaviorRequested()
   {
      return transitionRequested;
   }
}
