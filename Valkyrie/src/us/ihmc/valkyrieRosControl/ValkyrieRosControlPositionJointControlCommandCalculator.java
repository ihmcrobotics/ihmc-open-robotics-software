package us.ihmc.valkyrieRosControl;

import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.controllers.PIDController;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.math.filters.DeltaLimitedYoVariable;
import us.ihmc.valkyrieRosControl.dataHolders.YoPositionJointHandleHolder;

import java.util.Map;

public class ValkyrieRosControlPositionJointControlCommandCalculator
{
   private final YoPositionJointHandleHolder yoPositionJointHandleHolder;

   private final DeltaLimitedYoVariable positionStepSizeLimiter;

   private final DoubleYoVariable standPrepAngle;

   private final double controlDT;

   public ValkyrieRosControlPositionJointControlCommandCalculator(YoPositionJointHandleHolder yoPositionJointHandleHolder, Map<String, Double> gains,
         double standPrepAngle, double controlDT, YoVariableRegistry parentRegistry)
   {
      this.yoPositionJointHandleHolder = yoPositionJointHandleHolder;

      this.controlDT = controlDT;

      String positionJumpLimiterBaseName = yoPositionJointHandleHolder.getName();
      YoVariableRegistry registry = new YoVariableRegistry(positionJumpLimiterBaseName + "Command");

      this.standPrepAngle = new DoubleYoVariable(positionJumpLimiterBaseName + "StandPrepAngle", registry);

      this.positionStepSizeLimiter = new DeltaLimitedYoVariable(positionJumpLimiterBaseName + "PositionStepSizeLimiter", registry, 0.15);

      this.standPrepAngle.set(standPrepAngle);

      parentRegistry.addChild(registry);
   }

   public void initialize()
   {
      positionStepSizeLimiter.updateOutput(yoPositionJointHandleHolder.getQ(), yoPositionJointHandleHolder.getQ());
   }

   public void computeAndUpdateJointPosition(double inStateTime, double factor, double masterGain)
   {
      double standPrepFactor = 1.0 - factor;

      factor = MathTools.clipToMinMax(factor, 0.0, 1.0);

      double currentJointAngle = yoPositionJointHandleHolder.getQ();
      double standPrepDesired = standPrepAngle.getDoubleValue();

      double standPrepPosition = standPrepFactor * masterGain * standPrepDesired;
      double controllerPosition = factor * yoPositionJointHandleHolder.getControllerPositionDesired();

      double desiredPosition = standPrepPosition + controllerPosition;

      positionStepSizeLimiter.updateOutput(currentJointAngle, desiredPosition);

      yoPositionJointHandleHolder.setDesiredPosition(positionStepSizeLimiter.getDoubleValue());
   }
}
