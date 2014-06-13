package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates;

import java.util.Map;

import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.VariousWalkingManagers;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.VariousWalkingProviders;
import us.ihmc.commonWalkingControlModules.momentumBasedController.MomentumBasedController;
import us.ihmc.utilities.screwTheory.OneDoFJoint;

public class JointPDHighLevelHumanoidControllerFactory implements HighLevelBehaviorFactory
{
   private final boolean transitionRequested;
   private Map<OneDoFJoint, Double> initialKpGains;
   private Map<OneDoFJoint, Double> initialKdGains;

   public JointPDHighLevelHumanoidControllerFactory(Map<OneDoFJoint, Double> initialKpGains, Map<OneDoFJoint, Double> initialKdGains, boolean transitionRequested)
   {
      this.transitionRequested = transitionRequested;
      this.initialKpGains = initialKpGains;
      this.initialKdGains = initialKdGains;
   }

   @Override
   public HighLevelBehavior createHighLevelBehavior(VariousWalkingProviders variousWalkingProviders, VariousWalkingManagers variousWalkingManagers,
         MomentumBasedController momentumBasedController, ICPAndMomentumBasedController icpAndMomentumBasedController)
   {
      return new JointPDHighLevelHumanoidController(momentumBasedController.getYoTime(), initialKpGains, initialKdGains);
   }

   @Override
   public boolean isTransitionToBehaviorRequested()
   {
      return transitionRequested;
   }
}
