package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates;

import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.VariousWalkingManagers;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.VariousWalkingProviders;
import us.ihmc.commonWalkingControlModules.momentumBasedController.MomentumBasedController;

public class CarIngressEgressControllerFactory implements HighLevelBehaviorFactory
{
   private final boolean transitionRequested;
   private final WalkingControllerParameters multiContactControllerParameters;

   public CarIngressEgressControllerFactory(WalkingControllerParameters multiContactControllerParameters, boolean transitionRequested)
   {
      this.transitionRequested = transitionRequested;
      this.multiContactControllerParameters = multiContactControllerParameters;
   }

   @Override
   public HighLevelBehavior createHighLevelBehavior(VariousWalkingProviders variousWalkingProviders, VariousWalkingManagers variousWalkingManagers,
         MomentumBasedController momentumBasedController, ICPAndMomentumBasedController icpAndMomentumBasedController)
   {
      return new CarIngressEgressController(variousWalkingProviders, variousWalkingManagers, momentumBasedController, multiContactControllerParameters, null);
   }

   @Override
   public boolean isTransitionToBehaviorRequested()
   {
      return transitionRequested;
   }
}
