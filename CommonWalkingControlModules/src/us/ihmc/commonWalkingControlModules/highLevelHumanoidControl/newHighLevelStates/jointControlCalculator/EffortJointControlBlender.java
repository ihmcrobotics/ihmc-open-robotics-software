package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.newHighLevelStates.jointControlCalculator;

import us.ihmc.commonWalkingControlModules.controllerCore.command.lowLevel.LowLevelJointData;
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

   private final YoDouble tauOffset;
   private final YoDouble tauScale;

   public EffortJointControlBlender(String nameSuffix, OneDoFJoint oneDoFJoint, Map<String, Double> gains, double torqueOffset, YoVariableRegistry parentRegistry)
   {
      String namePrefix = oneDoFJoint.getName();
      YoVariableRegistry registry = new YoVariableRegistry(namePrefix + nameSuffix + "Command");

      this.tauOffset = new YoDouble("tau_offset_" + namePrefix + nameSuffix, registry);
      if (ENABLE_TAU_SCALE)
      {
         tauScale = new YoDouble("tau_scale_" + namePrefix + nameSuffix, registry);
         tauScale.set(1.0);
      }
      else
      {
         tauScale = null;
      }

      tauOffset.set(torqueOffset);

      parentRegistry.addChild(registry);
   }

   public void initialize()
   {
   }

   public double computeAndUpdateJointTorque(LowLevelJointData positionControllerDesireds, LowLevelJointData walkingControllerDesireds,
                                             double forceControlFactor)
   {
      forceControlFactor = MathTools.clamp(forceControlFactor, 0.0, 1.0);
      if (ENABLE_TAU_SCALE)
         forceControlFactor *= tauScale.getDoubleValue();

      double positionControlTau = (1.0 - forceControlFactor) * positionControllerDesireds.getDesiredTorque();
      double walkingControlTau = forceControlFactor * walkingControllerDesireds.getDesiredTorque();

      return positionControlTau + walkingControlTau + tauOffset.getDoubleValue();
   }

   public void subtractTorqueOffset(double torqueOffset)
   {
      tauOffset.sub(torqueOffset);
   }

}
