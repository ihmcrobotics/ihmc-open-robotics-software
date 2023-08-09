package us.ihmc.valkyrieRosControl;

import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.HighLevelControllerFactoryHelper;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.HighLevelControllerStateFactory;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.HighLevelControllerState;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HighLevelControllerName;
import us.ihmc.mecano.multiBodySystem.OneDoFJoint;
import us.ihmc.mecano.tools.MultiBodySystemTools;
import us.ihmc.robotics.partNames.HumanoidJointNameMap;
import us.ihmc.valkyrie.ValkyrieCalibrationParameters;
import us.ihmc.wholeBodyController.diagnostics.TorqueOffsetPrinter;

public class ValkyrieCalibrationControllerStateFactory implements HighLevelControllerStateFactory
{
   private final TorqueOffsetPrinter torqueOffsetPrinter;
   private final ValkyrieCalibrationParameters calibrationParameters;
   private final HumanoidJointNameMap jointNameMap;

   private ValkyrieCalibrationControllerState calibrationControllerState;

   public ValkyrieCalibrationControllerStateFactory(TorqueOffsetPrinter torqueOffsetPrinter, ValkyrieCalibrationParameters calibrationParameters, HumanoidJointNameMap jointNameMap)
   {
      this.torqueOffsetPrinter = torqueOffsetPrinter;
      this.calibrationParameters = calibrationParameters;
      this.jointNameMap = jointNameMap;
   }

   @Override
   public HighLevelControllerState getOrCreateControllerState(HighLevelControllerFactoryHelper controllerFactoryHelper)
   {
      if (calibrationControllerState != null)
         return calibrationControllerState;

      OneDoFJoint[] controlledJoints = MultiBodySystemTools.filterJoints(controllerFactoryHelper.getHighLevelHumanoidControllerToolbox().getControlledJoints(),
                                                                         OneDoFJoint.class);
      calibrationControllerState = new ValkyrieCalibrationControllerState(controllerFactoryHelper.getHighLevelHumanoidControllerToolbox(),
                                                                          controlledJoints,
                                                                          controllerFactoryHelper.getHighLevelHumanoidControllerToolbox().getYoTime(),
                                                                          jointNameMap,
                                                                          controllerFactoryHelper.getHighLevelControllerParameters(),
                                                                          controllerFactoryHelper.getLowLevelControllerOutput(),
                                                                          controllerFactoryHelper.getHighLevelHumanoidControllerToolbox().getFootContactStates(),
                                                                          controllerFactoryHelper.getHighLevelHumanoidControllerToolbox().getBipedSupportPolygons(),
                                                                          calibrationParameters,
                                                                          torqueOffsetPrinter);
      return calibrationControllerState;
   }

   public ValkyrieCalibrationControllerState getCalibrationControllerState()
   {
      return calibrationControllerState;
   }

   @Override
   public HighLevelControllerName getStateEnum()
   {
      return HighLevelControllerName.CALIBRATION;
   }

   @Override
   public boolean isTransitionToControllerRequested()
   {
      return false;
   }
}
