package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.newHighLevelStates.jointControlCalculator;

import us.ihmc.commonWalkingControlModules.controllerCore.command.lowLevel.LowLevelJointData;
import us.ihmc.commonWalkingControlModules.controllerCore.command.lowLevel.LowLevelJointDataReadOnly;
import us.ihmc.commonWalkingControlModules.controllerCore.command.lowLevel.LowLevelOneDoFJointDesiredDataHolder;
import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.controllers.PIDController;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

import java.util.Map;

public class EffortJointControlBlender
{
   /** This is for hardware debug purposes only. */
   private static final boolean ENABLE_TAU_SCALE = false;

   private final YoDouble tauScale;

   public EffortJointControlBlender(String nameSuffix, OneDoFJoint oneDoFJoint, YoVariableRegistry parentRegistry)
   {
      String namePrefix = oneDoFJoint.getName();
      YoVariableRegistry registry = new YoVariableRegistry(namePrefix + nameSuffix + "EffortJointControlBlender");

      if (ENABLE_TAU_SCALE)
      {
         tauScale = new YoDouble("tau_scale_" + namePrefix + nameSuffix, registry);
         tauScale.set(1.0);
      }
      else
      {
         tauScale = null;
      }

      parentRegistry.addChild(registry);
   }

   public void initialize()
   {
   }

   public void computeAndUpdateJointTorque(LowLevelJointData outputDataToPack, LowLevelJointDataReadOnly positionControllerDesireds,
                                             LowLevelJointDataReadOnly walkingControllerDesireds, double forceControlFactor)
   {
      forceControlFactor = MathTools.clamp(forceControlFactor, 0.0, 1.0);
      if (ENABLE_TAU_SCALE)
         forceControlFactor *= tauScale.getDoubleValue();

      double positionControlTau = (1.0 - forceControlFactor) * positionControllerDesireds.getDesiredTorque();
      double walkingControlTau = forceControlFactor * walkingControllerDesireds.getDesiredTorque();

      double controlTorque = positionControlTau + walkingControlTau;
      outputDataToPack.setDesiredTorque(controlTorque);
   }
}
