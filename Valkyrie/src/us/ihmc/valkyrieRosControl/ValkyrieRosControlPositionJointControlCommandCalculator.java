package us.ihmc.valkyrieRosControl;

import java.util.Map;

import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.math.filters.DeltaLimitedYoVariable;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.valkyrieRosControl.dataHolders.YoPositionJointHandleHolder;

public class ValkyrieRosControlPositionJointControlCommandCalculator
{
   private final YoPositionJointHandleHolder yoPositionJointHandleHolder;

   private final DeltaLimitedYoVariable positionStepSizeLimiter;

   private final DoubleYoVariable standPrepAngle;
   private final DoubleYoVariable initialAngle;

   public ValkyrieRosControlPositionJointControlCommandCalculator(YoPositionJointHandleHolder yoPositionJointHandleHolder, Map<String, Double> gains,
         double standPrepAngle, double controlDT, YoVariableRegistry parentRegistry)
   {
      this.yoPositionJointHandleHolder = yoPositionJointHandleHolder;

      String positionJumpLimiterBaseName = yoPositionJointHandleHolder.getName();
      YoVariableRegistry registry = new YoVariableRegistry(positionJumpLimiterBaseName + "Command");

      this.standPrepAngle = new DoubleYoVariable(positionJumpLimiterBaseName + "StandPrepAngle", registry);
      this.initialAngle = new DoubleYoVariable(positionJumpLimiterBaseName + "InitialAngle", registry);

      this.positionStepSizeLimiter = new DeltaLimitedYoVariable(positionJumpLimiterBaseName + "PositionStepSizeLimiter", registry, 0.15);

      this.standPrepAngle.set(standPrepAngle);

      parentRegistry.addChild(registry);
   }

   public void initialize()
   {
      positionStepSizeLimiter.updateOutput(yoPositionJointHandleHolder.getQ(), yoPositionJointHandleHolder.getQ());

      double q = yoPositionJointHandleHolder.getQ();
      OneDoFJoint joint = yoPositionJointHandleHolder.getOneDoFJoint();
      double jointLimitLower = joint.getJointLimitLower();
      double jointLimitUpper = joint.getJointLimitUpper();
      if (Double.isNaN(q) || Double.isInfinite(q))
         q = standPrepAngle.getDoubleValue();
      q = MathTools.clamp(q, jointLimitLower, jointLimitUpper);

      initialAngle.set(q);
   }

   public void computeAndUpdateJointPosition(double ramp, double factor, double masterGain)
   {
      double standPrepFactor = 1.0 - factor;

      factor = MathTools.clamp(factor, 0.0, 1.0);

      double currentJointAngle = yoPositionJointHandleHolder.getQ();
      double standPrepDesired = (1.0 - ramp) * initialAngle.getDoubleValue() + ramp * standPrepAngle.getDoubleValue();

      double standPrepPosition = standPrepFactor * masterGain * standPrepDesired;
      double controllerPosition = factor * yoPositionJointHandleHolder.getControllerPositionDesired();

      double desiredPosition = standPrepPosition + controllerPosition;

      positionStepSizeLimiter.updateOutput(currentJointAngle, desiredPosition);

      yoPositionJointHandleHolder.setDesiredPosition(positionStepSizeLimiter.getDoubleValue());
   }
}
