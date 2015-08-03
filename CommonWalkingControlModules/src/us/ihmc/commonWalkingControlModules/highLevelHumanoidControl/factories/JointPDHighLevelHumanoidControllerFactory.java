package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories;

import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.HighLevelBehavior;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.ICPAndMomentumBasedController;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.JointPDHighLevelHumanoidController;
import us.ihmc.commonWalkingControlModules.momentumBasedController.MomentumBasedController;

public class JointPDHighLevelHumanoidControllerFactory implements HighLevelBehaviorFactory
{
   private final boolean transitionRequested;
   private double initialKpGains;
   private double initialKdGains;

   public JointPDHighLevelHumanoidControllerFactory(double initialKpGains, double initialKdGains, boolean transitionRequested)
   {
      this.transitionRequested = transitionRequested;
      this.initialKpGains = initialKpGains;
      this.initialKdGains = initialKdGains;
   }

   @Override
   public HighLevelBehavior createHighLevelBehavior(VariousWalkingProviders variousWalkingProviders, VariousWalkingManagers variousWalkingManagers,
         MomentumBasedController momentumBasedController, ICPAndMomentumBasedController icpAndMomentumBasedController)
   {
      return new JointPDHighLevelHumanoidController(momentumBasedController, initialKpGains, initialKdGains);
   }

   @Override
   public boolean isTransitionToBehaviorRequested()
   {
      return transitionRequested;
   }
}
