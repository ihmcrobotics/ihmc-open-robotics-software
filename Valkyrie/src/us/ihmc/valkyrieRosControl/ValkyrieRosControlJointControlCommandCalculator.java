package us.ihmc.valkyrieRosControl;

import java.util.Map;

import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.controllers.PIDController;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.valkyrieRosControl.dataHolders.YoJointHandleHolder;

public class ValkyrieRosControlJointControlCommandCalculator
{
   private final YoJointHandleHolder yoJointHandleHolder;

   private final PIDController pidController;
   private final DoubleYoVariable tauOff;
   private final DoubleYoVariable standPrepAngle;

   private final double controlDT;

   public ValkyrieRosControlJointControlCommandCalculator(YoJointHandleHolder yoJointHandleHolder, Map<String, Double> gains, double torqueOffset,
         double standPrepAngle, double controlDT, YoVariableRegistry parentRegistry)
   {
      this.yoJointHandleHolder = yoJointHandleHolder;

      this.controlDT = controlDT;

      String pdControllerBaseName = yoJointHandleHolder.getName();
      YoVariableRegistry registry = new YoVariableRegistry(pdControllerBaseName + "Command");

      this.standPrepAngle = new DoubleYoVariable(pdControllerBaseName + "StandPrepAngle", registry);

      pidController = new PIDController(pdControllerBaseName + "StandPrep", registry);
      this.tauOff = new DoubleYoVariable("tau_offset_" + pdControllerBaseName, registry);

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
   }

   public void computeAndUpdateJointTorque(double inStateTime, double factor, double masterGain)
   {
      double standPrepFactor = 1.0 - factor;

      factor = MathTools.clipToMinMax(factor, 0.0, 1.0);

      double q = yoJointHandleHolder.getQ();
      double qDesired = standPrepAngle.getDoubleValue();
      double qd = yoJointHandleHolder.getQd();
      double qdDesired = 0.0;

      double standPrepTau = standPrepFactor * masterGain * pidController.compute(q, qDesired, qd, qdDesired, controlDT);
      double controllerTau = factor * yoJointHandleHolder.getControllerTauDesired();

      double desiredEffort = standPrepTau + controllerTau + tauOff.getDoubleValue();
      yoJointHandleHolder.setDesiredEffort(desiredEffort);
   }

   public void subtractTorqueOffset(double torqueOffset)
   {
      tauOff.sub(torqueOffset);
   }
}
