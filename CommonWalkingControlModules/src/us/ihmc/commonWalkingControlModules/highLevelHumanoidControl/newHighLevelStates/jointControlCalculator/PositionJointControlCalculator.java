package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.newHighLevelStates.jointControlCalculator;

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

   private final YoDouble standPrepAngle;
   private final YoDouble initialAngle;

   private final OneDoFJoint oneDoFJoint;

   public PositionJointControlCalculator(String namePrefix, OneDoFJoint oneDoFJoint, double standPrepAngle, YoVariableRegistry parentRegistry)
   {
      this.oneDoFJoint = oneDoFJoint;

      YoVariableRegistry registry = new YoVariableRegistry(namePrefix + "Command");

      this.standPrepAngle = new YoDouble(namePrefix + "StandPrepAngle", registry);
      this.initialAngle = new YoDouble(namePrefix + "InitialAngle", registry);

      this.positionStepSizeLimiter = new DeltaLimitedYoVariable(namePrefix + "PositionStepSizeLimiter", registry, 0.15);

      this.standPrepAngle.set(standPrepAngle);

      parentRegistry.addChild(registry);
   }

   public void initialize()
   {
      positionStepSizeLimiter.updateOutput(oneDoFJoint.getQ(), oneDoFJoint.getQ());

      double q = oneDoFJoint.getQ();
      double jointLimitLower = oneDoFJoint.getJointLimitLower();
      double jointLimitUpper = oneDoFJoint.getJointLimitUpper();
      if (Double.isNaN(q) || Double.isInfinite(q))
         q = standPrepAngle.getDoubleValue();
      q = MathTools.clamp(q, jointLimitLower, jointLimitUpper);

      initialAngle.set(q);
   }

   public double computeAndUpdateJointPosition(LowLevelOneDoFJointDesiredDataHolder positionControllerDesireds, LowLevelOneDoFJointDesiredDataHolder
                                           walkingControllerDesireds, double forceControlFactor, double masterPositionGain)
   {
      forceControlFactor = MathTools.clamp(forceControlFactor, 0.0, 1.0);

      double currentJointAngle = oneDoFJoint.getQ();

      double standPrepPosition = (1.0 - forceControlFactor) * masterPositionGain * positionControllerDesireds.getDesiredJointTorque(oneDoFJoint);
      double controllerPosition = forceControlFactor * walkingControllerDesireds.getDesiredJointPosition(oneDoFJoint);

      double desiredPosition = standPrepPosition + controllerPosition;

      positionStepSizeLimiter.updateOutput(currentJointAngle, desiredPosition);

      return positionStepSizeLimiter.getDoubleValue();
   }
}
