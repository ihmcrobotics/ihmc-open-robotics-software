package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.newHighLevelStates.jointControlCalculator;

import us.ihmc.commonWalkingControlModules.controllerCore.command.lowLevel.LowLevelJointData;
import us.ihmc.commonWalkingControlModules.controllerCore.command.lowLevel.LowLevelOneDoFJointDesiredDataHolder;
import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.controllers.PIDController;
import us.ihmc.robotics.math.filters.DeltaLimitedYoVariable;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

import java.util.Map;

public class PositionJointControlCalculator
{
   private final DeltaLimitedYoVariable positionStepSizeLimiter;
   private final YoDouble initialAngle;

   private final OneDoFJoint oneDoFJoint;

   public PositionJointControlCalculator(String nameSuffix, OneDoFJoint oneDoFJoint, YoVariableRegistry parentRegistry)
   {
      this.oneDoFJoint = oneDoFJoint;
      String namePrefix = oneDoFJoint.getName();
      YoVariableRegistry registry = new YoVariableRegistry(namePrefix + nameSuffix + "PositionJointControlCalculator");

      this.initialAngle = new YoDouble(namePrefix + nameSuffix + "InitialAngle", registry);

      this.positionStepSizeLimiter = new DeltaLimitedYoVariable(namePrefix + nameSuffix + "PositionStepSizeLimiter", registry, 0.15);

      parentRegistry.addChild(registry);
   }

   public void initialize()
   {
      positionStepSizeLimiter.updateOutput(oneDoFJoint.getQ(), oneDoFJoint.getQ());

      double q = oneDoFJoint.getQ();
      double jointLimitLower = oneDoFJoint.getJointLimitLower();
      double jointLimitUpper = oneDoFJoint.getJointLimitUpper();
      q = MathTools.clamp(q, jointLimitLower, jointLimitUpper);

      initialAngle.set(q);
   }

   public double computeAndUpdateJointPosition(LowLevelJointData positionControllerDesireds, double masterPositionGain)
   {
      double currentJointAngle = oneDoFJoint.getQ();
      double desiredPosition = masterPositionGain * positionControllerDesireds.getDesiredPosition();

      positionStepSizeLimiter.updateOutput(currentJointAngle, desiredPosition);
      desiredPosition = positionStepSizeLimiter.getDoubleValue();

      positionControllerDesireds.setDesiredPosition(desiredPosition);
      return desiredPosition;
   }
}
