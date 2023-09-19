package us.ihmc.valkyrieRosControl;

import us.ihmc.robotics.math.filters.DeltaLimitedYoVariable;
import us.ihmc.valkyrieRosControl.dataHolders.YoPositionJointHandleHolder;
import us.ihmc.yoVariables.registry.YoRegistry;

public class ValkyrieRosControlPositionJointControlCommandCalculator
{
   private final YoPositionJointHandleHolder yoPositionJointHandleHolder;

   private final DeltaLimitedYoVariable positionStepSizeLimiter;

   public ValkyrieRosControlPositionJointControlCommandCalculator(YoPositionJointHandleHolder yoPositionJointHandleHolder, YoRegistry parentRegistry)
   {
      this.yoPositionJointHandleHolder = yoPositionJointHandleHolder;

      String positionJumpLimiterBaseName = yoPositionJointHandleHolder.getName();
      YoRegistry registry = new YoRegistry(positionJumpLimiterBaseName + "Command");

      this.positionStepSizeLimiter = new DeltaLimitedYoVariable(positionJumpLimiterBaseName + "PositionStepSizeLimiter", registry, 0.15);

      parentRegistry.addChild(registry);
   }

   public void initialize()
   {
   }

   public void computeAndUpdateJointPosition()
   {
      yoPositionJointHandleHolder.updateControllerOutput();
      double currentJointAngle = yoPositionJointHandleHolder.getQ();
      double desiredPosition = yoPositionJointHandleHolder.getControllerPositionDesired();

      positionStepSizeLimiter.updateOutput(currentJointAngle, desiredPosition);

      yoPositionJointHandleHolder.setDesiredPosition(positionStepSizeLimiter.getDoubleValue());
   }
}
