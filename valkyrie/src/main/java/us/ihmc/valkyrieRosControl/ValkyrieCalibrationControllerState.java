package us.ihmc.valkyrieRosControl;

import us.ihmc.commonWalkingControlModules.configurations.HighLevelControllerParameters;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.stateEstimation.humanoid.kinematicsBasedStateEstimation.ForceSensorCalibrationModule;
import us.ihmc.wholeBodyController.diagnostics.JointTorqueOffsetEstimatorController;
import us.ihmc.wholeBodyController.diagnostics.TorqueOffsetPrinter;

public class ValkyrieCalibrationControllerState extends JointTorqueOffsetEstimatorController
{
   private ForceSensorCalibrationModule forceSensorCalibrationModule;

   public ValkyrieCalibrationControllerState(HighLevelHumanoidControllerToolbox highLevelControllerToolbox,
                                             HighLevelControllerParameters highLevelControllerParameters,
                                             TorqueOffsetPrinter torqueOffsetPrinter)
   {
      super(highLevelControllerToolbox, highLevelControllerParameters, torqueOffsetPrinter);
   }

   public void attachForceSensorCalibrationModule(ForceSensorCalibrationModule forceSensorCalibrationModule)
   {
      this.forceSensorCalibrationModule = forceSensorCalibrationModule;
   }

   @Override
   public void doTransitionOutOfAction()
   {
      super.doTransitionOutOfAction();

      if (forceSensorCalibrationModule != null)
         forceSensorCalibrationModule.requestFootForceSensorCalibrationAtomic();
   }
}
