package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.newHighLevelStates.jointControlCalculator;

import us.ihmc.commonWalkingControlModules.controllerCore.command.lowLevel.LowLevelJointData;
import us.ihmc.commonWalkingControlModules.controllerCore.command.lowLevel.LowLevelJointDataReadOnly;
import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.math.filters.DeltaLimitedYoVariable;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class JointControlBlender
{
   /** This is for hardware debug purposes only. */
   private static final boolean ENABLE_TAU_SCALE = false;

   private final YoDouble tauScale;
   private final DeltaLimitedYoVariable positionStepSizeLimiter;

   private final OneDoFJoint oneDoFJoint;

   public JointControlBlender(String nameSuffix, OneDoFJoint oneDoFJoint, YoVariableRegistry parentRegistry)
   {
      this.oneDoFJoint = oneDoFJoint;
      String namePrefix = oneDoFJoint.getName();

      YoVariableRegistry registry = new YoVariableRegistry(namePrefix + nameSuffix + "JointControlBlender");

      this.positionStepSizeLimiter = new DeltaLimitedYoVariable(namePrefix + "PositionStepSizeLimiter", registry, 0.15);

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
      double q = oneDoFJoint.getQ();
      positionStepSizeLimiter.updateOutput(q, q);
   }

   public void computeAndUpdateJointControl(LowLevelJointData outputDataToPack, LowLevelJointDataReadOnly positionControllerDesireds,
                                               LowLevelJointDataReadOnly walkingControllerDesireds,
                                               double forceControlBlendingFactor)
   {
      forceControlBlendingFactor = MathTools.clamp(forceControlBlendingFactor, 0.0, 1.0);

      if (ENABLE_TAU_SCALE)
         forceControlBlendingFactor *= tauScale.getDoubleValue();

      double standPrepPosition = (1.0 - forceControlBlendingFactor) * positionControllerDesireds.getDesiredTorque();
      double controllerPosition = forceControlBlendingFactor * walkingControllerDesireds.getDesiredTorque();
      double positionControlTau = (1.0 - forceControlBlendingFactor) * positionControllerDesireds.getDesiredTorque();
      double walkingControlTau = forceControlBlendingFactor * walkingControllerDesireds.getDesiredTorque();

      double desiredPosition = standPrepPosition + controllerPosition;
      double controlTorque = positionControlTau + walkingControlTau;

      double currentJointAngle = oneDoFJoint.getQ();
      positionStepSizeLimiter.updateOutput(currentJointAngle, desiredPosition);

      outputDataToPack.setDesiredPosition(positionStepSizeLimiter.getDoubleValue());
      outputDataToPack.setDesiredTorque(controlTorque);
   }
}
