package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.newHighLevelStates.jointControlCalculator;

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

   private final PIDController pidPositionController;
   private final YoDouble tauOffset;
   private final YoDouble tauScale;
   private final YoDouble standPrepAngle;
   private final YoDouble initialAngle;

   private final OneDoFJoint oneDoFJoint;
   private final double controlDT;

   public EffortJointControlBlender(String namePrefix, OneDoFJoint oneDoFJoint, Map<String, Double> gains, double torqueOffset, double standPrepAngle, double controlDT, YoVariableRegistry parentRegistry)
   {
      this.controlDT = controlDT;
      this.oneDoFJoint = oneDoFJoint;

      YoVariableRegistry registry = new YoVariableRegistry(namePrefix + "Command");

      this.standPrepAngle = new YoDouble(namePrefix + "StandPrepAngle", registry);
      this.initialAngle = new YoDouble(namePrefix + "InitialAngle", registry);

      pidPositionController = new PIDController(namePrefix + "StandPrep", registry);
      this.tauOffset = new YoDouble("tau_offset_" + namePrefix, registry);
      if (ENABLE_TAU_SCALE)
      {
         tauScale = new YoDouble("tau_scale_" + namePrefix, registry);
         tauScale.set(1.0);
      }
      else
      {
         tauScale = null;
      }

      pidPositionController.setProportionalGain(gains.get("kp"));
      pidPositionController.setDerivativeGain(gains.get("kd"));
      pidPositionController.setIntegralGain(gains.get("ki"));
      pidPositionController.setMaxIntegralError(50.0);
      pidPositionController.setCumulativeError(0.0);

      tauOffset.set(torqueOffset);

      this.standPrepAngle.set(standPrepAngle);

      parentRegistry.addChild(registry);
   }

   public void initialize()
   {
      pidPositionController.setCumulativeError(0.0);

      double q = oneDoFJoint.getQ();
      double jointLimitLower = oneDoFJoint.getJointLimitLower();
      double jointLimitUpper = oneDoFJoint.getJointLimitUpper();
      if (Double.isNaN(q) || Double.isInfinite(q))
         q = standPrepAngle.getDoubleValue();
      q = MathTools.clamp(q, jointLimitLower, jointLimitUpper);

      initialAngle.set(q);
   }

   public double computeAndUpdateJointTorque(LowLevelOneDoFJointDesiredDataHolder positionControllerDesireds, LowLevelOneDoFJointDesiredDataHolder
                                           walkingControllerDesireds, double forceControlFactor, double masterPositionGain)
   {
      forceControlFactor = MathTools.clamp(forceControlFactor, 0.0, 1.0);
      if (ENABLE_TAU_SCALE)
         forceControlFactor *= tauScale.getDoubleValue();

      double q = oneDoFJoint.getQ();
      double qDesired = positionControllerDesireds.getDesiredJointPosition(oneDoFJoint);
      double qd = oneDoFJoint.getQd();
      double qdDesired = positionControllerDesireds.getDesiredJointVelocity(oneDoFJoint);

      double positionControlTau = (1.0 - forceControlFactor) * masterPositionGain * pidPositionController.compute(q, qDesired, qd, qdDesired, controlDT);
      double walkingControlTau = forceControlFactor * walkingControllerDesireds.getDesiredJointTorque(oneDoFJoint);

      double desiredEffort = positionControlTau + walkingControlTau + tauOffset.getDoubleValue();

      return desiredEffort;
   }

   public void subtractTorqueOffset(double torqueOffset)
   {
      tauOffset.sub(torqueOffset);
   }

}
