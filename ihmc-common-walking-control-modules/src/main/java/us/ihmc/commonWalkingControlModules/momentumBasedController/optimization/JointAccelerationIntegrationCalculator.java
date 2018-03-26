package us.ihmc.commonWalkingControlModules.momentumBasedController.optimization;

import java.util.ArrayList;
import java.util.List;

import gnu.trove.list.array.TDoubleArrayList;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.JointAccelerationIntegrationCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.lowLevel.LowLevelOneDoFJointDesiredDataHolder;
import us.ihmc.commonWalkingControlModules.controllerCore.parameters.JointAccelerationIntegrationParametersReadOnly;
import us.ihmc.commons.MathTools;
import us.ihmc.commons.PrintTools;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutput;

public class JointAccelerationIntegrationCalculator
{
   public static final double DEFAULT_ALPHA_POSITION = 0.9996;
   public static final double DEFAULT_ALPHA_VELOCITY = 0.95;
   public static final double DEFAULT_MAX_POSITION_ERROR = 0.2;
   public static final double DEFAULT_MAX_VELOCITY = 2.0;

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final List<OneDoFJoint> jointsToComputeDesiredPositionFor = new ArrayList<>();
   private final TDoubleArrayList jointSpecificAlphaPosition = new TDoubleArrayList();
   private final TDoubleArrayList jointSpecificAlphaVelocity = new TDoubleArrayList();
   private final TDoubleArrayList jointSpecificMaxPositionError = new TDoubleArrayList();
   private final TDoubleArrayList jointSpecificMaxVelocity = new TDoubleArrayList();

   private final YoDouble defaultAlphaPositionIntegration = new YoDouble("defaultAlphaPositionIntegration", registry);
   private final YoDouble defaultAlphaVelocityIntegration = new YoDouble("defaultAlphaVelocityIntegration", registry);
   private final YoDouble defaultIntegrationMaxVelocity = new YoDouble("defaultIntegrationMaxVelocity", registry);
   private final YoDouble defaultIntegrationMaxPositionError = new YoDouble("defaultIntegrationMaxPositionError", registry);

   private final double controlDT;

   public JointAccelerationIntegrationCalculator(double controlDT, YoVariableRegistry parentRegistry)
   {
      this.controlDT = controlDT;
      defaultAlphaPositionIntegration.set(DEFAULT_ALPHA_POSITION);
      defaultAlphaVelocityIntegration.set(DEFAULT_ALPHA_VELOCITY);
      defaultIntegrationMaxPositionError.set(DEFAULT_MAX_POSITION_ERROR);
      defaultIntegrationMaxVelocity.set(DEFAULT_MAX_VELOCITY);

      parentRegistry.addChild(registry);
   }

   public void submitJointAccelerationIntegrationCommand(JointAccelerationIntegrationCommand command)
   {
      for (int commandJointIndex = 0; commandJointIndex < command.getNumberOfJointsToComputeDesiredPositionFor(); commandJointIndex++)
      {
         OneDoFJoint jointToComputeDesierdPositionFor = command.getJointToComputeDesiredPositionFor(commandJointIndex);
         int localJointIndex = jointsToComputeDesiredPositionFor.indexOf(jointToComputeDesierdPositionFor);
         JointAccelerationIntegrationParametersReadOnly jointParameters = command.getJointParameters(commandJointIndex);

         double newAlphaPosition = jointParameters.getAlphaPosition();
         if (Double.isNaN(newAlphaPosition) || !MathTools.intervalContains(newAlphaPosition, 0.0, 1.0))
            newAlphaPosition = defaultAlphaPositionIntegration.getDoubleValue();

         double newAlphaVelocity = jointParameters.getAlphaVelocity();
         if (Double.isNaN(newAlphaVelocity) || !MathTools.intervalContains(newAlphaVelocity, 0.0, 1.0))
            newAlphaVelocity = defaultAlphaVelocityIntegration.getDoubleValue();

         double newMaxPositionError = jointParameters.getMaxPositionError();
         if (Double.isNaN(newMaxPositionError) || newMaxPositionError < 0.0)
            newMaxPositionError = defaultIntegrationMaxPositionError.getDoubleValue();

         double newMaxVelocity = jointParameters.getMaxVelocity();
         if (Double.isNaN(newMaxVelocity) || newMaxVelocity < 0.0)
            newMaxVelocity = defaultIntegrationMaxVelocity.getDoubleValue();

         if (localJointIndex < 0)
         {
            jointsToComputeDesiredPositionFor.add(jointToComputeDesierdPositionFor);
            jointSpecificAlphaPosition.add(newAlphaPosition);
            jointSpecificAlphaVelocity.add(newAlphaVelocity);
            jointSpecificMaxPositionError.add(newMaxVelocity);
            jointSpecificMaxVelocity.add(newMaxVelocity);
         }
         else
         {
            jointSpecificAlphaPosition.set(localJointIndex, newAlphaPosition);
            jointSpecificAlphaVelocity.set(localJointIndex, newAlphaVelocity);
            jointSpecificMaxPositionError.set(localJointIndex, newMaxVelocity);
            jointSpecificMaxVelocity.set(localJointIndex, newMaxVelocity);
         }
      }
   }

   public void computeAndUpdateDataHolder(LowLevelOneDoFJointDesiredDataHolder lowLevelJointDataHolderToUpdate)
   {
      for (int jointIndex = 0; jointIndex < jointsToComputeDesiredPositionFor.size(); jointIndex++)
      {
         OneDoFJoint joint = jointsToComputeDesiredPositionFor.get(jointIndex);

         JointDesiredOutput lowLevelJointData = lowLevelJointDataHolderToUpdate.getJointDesiredOutput(joint);
         if (lowLevelJointData == null || !lowLevelJointData.hasDesiredAcceleration())
        	 continue;
         if (!lowLevelJointData.hasDesiredVelocity())
            lowLevelJointData.setDesiredVelocity(joint.getQd());
         if (!lowLevelJointData.hasDesiredPosition())
            lowLevelJointData.setDesiredPosition(joint.getQ());

         double desiredAcceleration = lowLevelJointData.getDesiredAcceleration();
         double desiredVelocity = lowLevelJointData.getDesiredVelocity();
         double desiredPosition = lowLevelJointData.getDesiredPosition();

         double alphaPosition = jointSpecificAlphaPosition.get(jointIndex);
         double alphaVelocity = jointSpecificAlphaVelocity.get(jointIndex);
         double maxPositionError = jointSpecificMaxPositionError.get(jointIndex);
         double maxVelocity = jointSpecificMaxVelocity.get(jointIndex);

         desiredVelocity *= alphaVelocity;
         desiredVelocity += desiredAcceleration * controlDT;
         desiredVelocity = MathTools.clamp(desiredVelocity, maxVelocity);
         desiredPosition += desiredVelocity * controlDT;

         double errorPosition = MathTools.clamp(desiredPosition - joint.getQ(), maxPositionError);
         desiredPosition = joint.getQ() + errorPosition;
         desiredPosition = MathTools.clamp(desiredPosition, joint.getJointLimitLower(), joint.getJointLimitUpper());
         desiredPosition = alphaPosition * desiredPosition + (1.0 - alphaPosition) * joint.getQ();
         desiredVelocity = (desiredPosition - lowLevelJointData.getDesiredPosition()) / controlDT;

         lowLevelJointData.setDesiredVelocity(desiredVelocity);
         lowLevelJointData.setDesiredPosition(desiredPosition);
      }
   }
}
