package us.ihmc.valkyrieRosControl;

import us.ihmc.robotics.math.filters.DeltaLimitedYoVariable;
import us.ihmc.valkyrieRosControl.dataHolders.YoPositionJointHandleHolder;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public class ValkyrieRosControlPositionJointControlCommandCalculator
{
   private final YoPositionJointHandleHolder yoPositionJointHandleHolder;

   private final DeltaLimitedYoVariable positionStepSizeLimiter;

   public ValkyrieRosControlPositionJointControlCommandCalculator(YoPositionJointHandleHolder yoPositionJointHandleHolder, YoVariableRegistry parentRegistry)
   {
      this.yoPositionJointHandleHolder = yoPositionJointHandleHolder;

      String positionJumpLimiterBaseName = yoPositionJointHandleHolder.getName();
      YoVariableRegistry registry = new YoVariableRegistry(positionJumpLimiterBaseName + "Command");

      this.positionStepSizeLimiter = new DeltaLimitedYoVariable(positionJumpLimiterBaseName + "PositionStepSizeLimiter", registry, 0.15);

      parentRegistry.addChild(registry);
   }

   public void initialize()
   {
   }

   public void computeAndUpdateJointPosition()
   {
      double currentJointAngle = yoPositionJointHandleHolder.getQ();
      double desiredPosition = yoPositionJointHandleHolder.getControllerPositionDesired();

      positionStepSizeLimiter.updateOutput(currentJointAngle, desiredPosition);

      yoPositionJointHandleHolder.setDesiredPosition(positionStepSizeLimiter.getDoubleValue());
   }
}
