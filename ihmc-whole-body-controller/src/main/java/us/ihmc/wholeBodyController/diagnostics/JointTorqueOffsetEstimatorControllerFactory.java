package us.ihmc.wholeBodyController.diagnostics;

import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.HighLevelControllerFactoryHelper;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.HighLevelBehaviorFactory;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.HighLevelControlManagerFactory;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.HighLevelControllerStateFactory;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.HighLevelHumanoidControllerFactory;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.HighLevelBehavior;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.HighLevelControllerState;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HighLevelController;

public class JointTorqueOffsetEstimatorControllerFactory implements HighLevelControllerStateFactory
{
   private final TorqueOffsetPrinter torqueOffsetPrinter;
   private JointTorqueOffsetEstimatorController jointTorqueOffsetEstimatorController;

   public JointTorqueOffsetEstimatorControllerFactory(TorqueOffsetPrinter torqueOffsetPrinter)
   {
      this.torqueOffsetPrinter = torqueOffsetPrinter;
   }

   @Override
   public HighLevelControllerState getOrCreateControllerState(HighLevelControllerFactoryHelper controllerFactoryHelper)
   {
      if (jointTorqueOffsetEstimatorController != null)
         return jointTorqueOffsetEstimatorController;

      jointTorqueOffsetEstimatorController = new JointTorqueOffsetEstimatorController(controllerFactoryHelper.getHighLevelHumanoidControllerToolbox(), torqueOffsetPrinter);
      return jointTorqueOffsetEstimatorController;
   }

   public JointTorqueOffsetEstimatorController getJointTorqueOffsetEstimatorController()
   {
      return jointTorqueOffsetEstimatorController;
   }

   @Override
   public HighLevelController getStateEnum()
   {
      return HighLevelController.CALIBRATION;
   }

   @Override
   public boolean isTransitionToControllerRequested()
   {
      return false;
   }
}
