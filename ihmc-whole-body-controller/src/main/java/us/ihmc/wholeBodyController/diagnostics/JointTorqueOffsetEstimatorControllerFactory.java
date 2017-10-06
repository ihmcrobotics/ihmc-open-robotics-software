package us.ihmc.wholeBodyController.diagnostics;

import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.HighLevelBehaviorFactory;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.HighLevelControlManagerFactory;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.HighLevelBehavior;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;

public class JointTorqueOffsetEstimatorControllerFactory implements HighLevelBehaviorFactory
{
   private final TorqueOffsetPrinter torqueOffsetPrinter;
   private JointTorqueOffsetEstimatorController jointTorqueOffsetEstimatorController;

   public JointTorqueOffsetEstimatorControllerFactory(TorqueOffsetPrinter torqueOffsetPrinter)
   {
      this.torqueOffsetPrinter = torqueOffsetPrinter;
   }

   @Override
   public HighLevelBehavior createHighLevelBehavior(HighLevelControlManagerFactory variousWalkingManagers,
         HighLevelHumanoidControllerToolbox highLevelControllerToolbox)
   {
      jointTorqueOffsetEstimatorController = new JointTorqueOffsetEstimatorController(highLevelControllerToolbox, torqueOffsetPrinter);
      return jointTorqueOffsetEstimatorController;
   }

   public JointTorqueOffsetEstimatorController getJointTorqueOffsetEstimatorController()
   {
      return jointTorqueOffsetEstimatorController;
   }

   @Override
   public boolean isTransitionToBehaviorRequested()
   {
      return false;
   }
}
