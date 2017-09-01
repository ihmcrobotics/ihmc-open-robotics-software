package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.newHighLevelStates.jointControlCalculator;

import us.ihmc.commonWalkingControlModules.controllerCore.command.lowLevel.LowLevelJointData;
import us.ihmc.commonWalkingControlModules.controllerCore.command.lowLevel.LowLevelJointDataReadOnly;
import us.ihmc.commonWalkingControlModules.controllerCore.command.lowLevel.LowLevelOneDoFJointDesiredDataHolder;
import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.math.filters.DeltaLimitedYoVariable;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class PositionJointControlBlender
{
   private final DeltaLimitedYoVariable positionStepSizeLimiter;

   private final OneDoFJoint oneDoFJoint;

   public PositionJointControlBlender(String nameSuffix, OneDoFJoint oneDoFJoint, YoVariableRegistry parentRegistry)
   {
      this.oneDoFJoint = oneDoFJoint;
      String namePrefix = oneDoFJoint.getName();

      YoVariableRegistry registry = new YoVariableRegistry(namePrefix + nameSuffix + "PositionJointControlBlender");

      this.positionStepSizeLimiter = new DeltaLimitedYoVariable(namePrefix + "PositionStepSizeLimiter", registry, 0.15);

      parentRegistry.addChild(registry);
   }

   public void initialize()
   {
      double q = oneDoFJoint.getQ();
      positionStepSizeLimiter.updateOutput(q, q);
   }

   public void computeAndUpdateJointPosition(LowLevelJointData jointDataToPack, LowLevelJointDataReadOnly positionControllerDesireds,
                                               LowLevelJointDataReadOnly walkingControllerDesireds,
                                               double forceControlBlendingFactor)
   {
      forceControlBlendingFactor = MathTools.clamp(forceControlBlendingFactor, 0.0, 1.0);

      double currentJointAngle = oneDoFJoint.getQ();

      double standPrepPosition = (1.0 - forceControlBlendingFactor) * positionControllerDesireds.getDesiredTorque();
      double controllerPosition = forceControlBlendingFactor * walkingControllerDesireds.getDesiredTorque();

      double desiredPosition = standPrepPosition + controllerPosition;

      positionStepSizeLimiter.updateOutput(currentJointAngle, desiredPosition);

      jointDataToPack.setDesiredPosition(positionStepSizeLimiter.getDoubleValue());
   }
}
