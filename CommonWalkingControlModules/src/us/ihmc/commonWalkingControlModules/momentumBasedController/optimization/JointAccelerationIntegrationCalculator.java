package us.ihmc.commonWalkingControlModules.momentumBasedController.optimization;

import java.util.ArrayList;
import java.util.List;

import gnu.trove.list.array.TDoubleArrayList;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.JointAccelerationIntegrationCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.lowLevel.LowLevelJointData;
import us.ihmc.commonWalkingControlModules.controllerCore.command.lowLevel.LowLevelOneDoFJointDesiredDataHolder;
import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.screwTheory.OneDoFJoint;

public class JointAccelerationIntegrationCalculator
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final List<OneDoFJoint> jointsToComputeDesiredPositionFor = new ArrayList<>();
   private final TDoubleArrayList jointSpecificAlphaPosition = new TDoubleArrayList();
   private final TDoubleArrayList jointSpecificAlphaVelocity = new TDoubleArrayList();
   private final TDoubleArrayList jointSpecificMaxPositionError = new TDoubleArrayList();
   private final TDoubleArrayList jointSpecificMaxVelocity = new TDoubleArrayList();

   private final DoubleYoVariable defaultAlphaPositionIntegration = new DoubleYoVariable("defaultAlphaPositionIntegration", registry);
   private final DoubleYoVariable defaultAlphaVelocityIntegration = new DoubleYoVariable("defaultAlphaVelocityIntegration", registry);
   private final DoubleYoVariable defaultIntegrationMaxVelocity = new DoubleYoVariable("defaultIntegrationMaxVelocity", registry);
   private final DoubleYoVariable defaultIntegrationMaxPositionError = new DoubleYoVariable("defaultIntegrationMaxPositionError", registry);

   private final double controlDT;

   public JointAccelerationIntegrationCalculator(double controlDT, YoVariableRegistry parentRegistry)
   {
      this.controlDT = controlDT;
      defaultAlphaPositionIntegration.set(0.9996);
      defaultAlphaVelocityIntegration.set(0.95);
      defaultIntegrationMaxPositionError.set(0.2);
      defaultIntegrationMaxVelocity.set(2.0);

      parentRegistry.addChild(registry);
   }

   public void submitJointAccelerationIntegrationCommand(JointAccelerationIntegrationCommand command)
   {
      for (int commandJointIndex = 0; commandJointIndex < command.getNumberOfJointsToComputeDesiredPositionFor(); commandJointIndex++)
      {
         OneDoFJoint jointToComputeDesierdPositionFor = command.getJointToComputeDesiredPositionFor(commandJointIndex);
         int localJointIndex = jointsToComputeDesiredPositionFor.indexOf(jointToComputeDesierdPositionFor);

         double newAlphaPosition = command.getJointAlphaPosition(commandJointIndex);
         if (Double.isNaN(newAlphaPosition) || !MathTools.intervalContains(newAlphaPosition, 0.0, 1.0))
            newAlphaPosition = defaultAlphaPositionIntegration.getDoubleValue();

         double newAlphaVelocity = command.getJointAlphaVelocity(commandJointIndex);
         if (Double.isNaN(newAlphaVelocity) || !MathTools.intervalContains(newAlphaVelocity, 0.0, 1.0))
            newAlphaVelocity = defaultAlphaVelocityIntegration.getDoubleValue();

         double newMaxPositionError = command.getJointMaxPositionError(commandJointIndex);
         if (Double.isNaN(newMaxPositionError) || newMaxPositionError < 0.0)
            newMaxPositionError = defaultIntegrationMaxPositionError.getDoubleValue();

         double newMaxVelocity = command.getJointMaxVelocity(commandJointIndex);
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

         LowLevelJointData lowLevelJointData = lowLevelJointDataHolderToUpdate.getLowLevelJointData(joint);
         if (lowLevelJointData == null)
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
