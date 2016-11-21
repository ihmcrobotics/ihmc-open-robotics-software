package us.ihmc.valkyrieRosControl;

import java.util.Map;

import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.controllers.PIDController;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.valkyrieRosControl.dataHolders.YoEffortJointHandleHolder;

public class ValkyrieRosControlEffortJointControlCommandCalculator
{
   /** This is for hardware debug purposes only. */
   private static final boolean ENABLE_TAU_SCALE = false;

   private final YoEffortJointHandleHolder yoEffortJointHandleHolder;

   private final PIDController pidController;
   private final DoubleYoVariable tauOff;
   private final DoubleYoVariable tauScale;
   private final DoubleYoVariable standPrepAngle;
   private final DoubleYoVariable initialAngle;

   private final double controlDT;

   public ValkyrieRosControlEffortJointControlCommandCalculator(YoEffortJointHandleHolder yoEffortJointHandleHolder, Map<String, Double> gains, double torqueOffset,
         double standPrepAngle, double controlDT, YoVariableRegistry parentRegistry)
   {
      this.yoEffortJointHandleHolder = yoEffortJointHandleHolder;

      this.controlDT = controlDT;

      String pdControllerBaseName = yoEffortJointHandleHolder.getName();
      YoVariableRegistry registry = new YoVariableRegistry(pdControllerBaseName + "Command");

      this.standPrepAngle = new DoubleYoVariable(pdControllerBaseName + "StandPrepAngle", registry);
      this.initialAngle = new DoubleYoVariable(pdControllerBaseName + "InitialAngle", registry);

      pidController = new PIDController(pdControllerBaseName + "StandPrep", registry);
      this.tauOff = new DoubleYoVariable("tau_offset_" + pdControllerBaseName, registry);
      if (ENABLE_TAU_SCALE)
      {
         tauScale = new DoubleYoVariable("tau_scale_" + pdControllerBaseName, registry);
         tauScale.set(1.0);
      }
      else
      {
         tauScale = null;
      }

      pidController.setProportionalGain(gains.get("kp"));
      pidController.setDerivativeGain(gains.get("kd"));
      pidController.setIntegralGain(gains.get("ki"));
      pidController.setMaxIntegralError(50.0);
      pidController.setCumulativeError(0.0);

      tauOff.set(torqueOffset);

      this.standPrepAngle.set(standPrepAngle);

      parentRegistry.addChild(registry);
   }

   public void initialize()
   {
      pidController.setCumulativeError(0.0);

      double q = yoEffortJointHandleHolder.getQ();
      OneDoFJoint joint = yoEffortJointHandleHolder.getOneDoFJoint();
      double jointLimitLower = joint.getJointLimitLower();
      double jointLimitUpper = joint.getJointLimitUpper();
      if (Double.isNaN(q) || Double.isInfinite(q))
         q = standPrepAngle.getDoubleValue();
      q = MathTools.clipToMinMax(q, jointLimitLower, jointLimitUpper);

      initialAngle.set(q);
   }

   public void computeAndUpdateJointTorque(double ramp, double factor, double masterGain)
   {
      double standPrepFactor = 1.0 - factor;

      factor = MathTools.clipToMinMax(factor, 0.0, 1.0);
      if (ENABLE_TAU_SCALE)
         factor *= tauScale.getDoubleValue();

      double q = yoEffortJointHandleHolder.getQ();
      double qDesired = (1.0 - ramp) * initialAngle.getDoubleValue() + ramp * standPrepAngle.getDoubleValue();
      double qd = yoEffortJointHandleHolder.getQd();
      double qdDesired = 0.0;

      double standPrepTau = standPrepFactor * masterGain * pidController.compute(q, qDesired, qd, qdDesired, controlDT);
      double controllerTau = factor * yoEffortJointHandleHolder.getControllerTauDesired();

      double desiredEffort = standPrepTau + controllerTau + tauOff.getDoubleValue();
      yoEffortJointHandleHolder.setDesiredEffort(desiredEffort);
   }

   public void subtractTorqueOffset(double torqueOffset)
   {
      tauOff.sub(torqueOffset);
   }
}
