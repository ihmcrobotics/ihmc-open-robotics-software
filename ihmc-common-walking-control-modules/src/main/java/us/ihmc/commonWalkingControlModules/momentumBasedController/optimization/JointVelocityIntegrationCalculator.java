package us.ihmc.commonWalkingControlModules.momentumBasedController.optimization;

import gnu.trove.list.array.TDoubleArrayList;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.JointAccelerationIntegrationCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics.JointVelocityIntegrationCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.lowLevel.LowLevelOneDoFJointDesiredDataHolder;
import us.ihmc.commonWalkingControlModules.controllerCore.parameters.JointAccelerationIntegrationParametersReadOnly;
import us.ihmc.commonWalkingControlModules.controllerCore.parameters.JointVelocityIntegrationParametersReadOnly;
import us.ihmc.commons.MathTools;
import us.ihmc.commons.PrintTools;
import us.ihmc.robotics.math.filters.AlphaFilteredYoVariable;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutput;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

import java.util.ArrayList;
import java.util.List;

public class JointVelocityIntegrationCalculator
{
   public static final double DEFAULT_POSITION_BREAK_FREQUENCY = Double.POSITIVE_INFINITY;
   public static final double DEFAULT_ACCELERATION_BREAK_FREQUENCY = 5.0;
   public static final double DEFAULT_MAX_POSITION_ERROR = 0.2;
   public static final double DEFAULT_MAX_ACCELERATION = 20.0;

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final List<OneDoFJoint> jointsToComputeDesiredPositionFor = new ArrayList<>();
   private final TDoubleArrayList jointSpecificPositionBreakFrequency = new TDoubleArrayList();
   private final TDoubleArrayList jointSpecificAccelerationBreakFrequency = new TDoubleArrayList();
   private final TDoubleArrayList jointSpecificMaxPositionError = new TDoubleArrayList();
   private final TDoubleArrayList jointSpecificMaxAcceleration = new TDoubleArrayList();

   private final YoDouble defaultPositionBreakFrequency = new YoDouble("defaultPositionBreakFrequencyIntegration", registry);
   private final YoDouble defaultAccelerationBreakFrequency = new YoDouble("defaultAccelerationBreakFrequencyDifferentiation", registry);
   private final YoDouble defaultIntegrationMaxAcceleration = new YoDouble("defaultIntegrationMaxAcceleration", registry);
   private final YoDouble defaultIntegrationMaxPositionError = new YoDouble("defaultIntegrationMaxPositionError", registry);

   private TDoubleArrayList previousDesiredVelocity = new TDoubleArrayList();
   private final double controlDT;

   public JointVelocityIntegrationCalculator(double controlDT, YoVariableRegistry parentRegistry)
   {
      this.controlDT = controlDT;
      defaultPositionBreakFrequency.set(DEFAULT_POSITION_BREAK_FREQUENCY);
      defaultAccelerationBreakFrequency.set(DEFAULT_ACCELERATION_BREAK_FREQUENCY);
      defaultIntegrationMaxPositionError.set(DEFAULT_MAX_POSITION_ERROR);
      defaultIntegrationMaxAcceleration.set(DEFAULT_MAX_ACCELERATION);

      parentRegistry.addChild(registry);
   }

   public void reset()
   {
      previousDesiredVelocity.clear();
   }

   public void submitJointVelocityIntegrationCommand(JointVelocityIntegrationCommand command)
   {
      for (int commandJointIndex = 0; commandJointIndex < command.getNumberOfJointsToComputeDesiredPositionFor(); commandJointIndex++)
      {
         OneDoFJoint jointToComputeDesiredPositionFor = command.getJointToComputeDesiredPositionFor(commandJointIndex);
         int localJointIndex = jointsToComputeDesiredPositionFor.indexOf(jointToComputeDesiredPositionFor);
         JointVelocityIntegrationParametersReadOnly jointParameters = command.getJointParameters(commandJointIndex);

         double newPositionBreakFrequency = jointParameters.getPositionBreakFrequency();
         if (Double.isNaN(newPositionBreakFrequency) || newPositionBreakFrequency < 0.0)
            newPositionBreakFrequency = defaultPositionBreakFrequency.getDoubleValue();

         double newAccelerationBreakFrequency = jointParameters.getAccelerationBreakFrequency();
         if (Double.isNaN(newAccelerationBreakFrequency) || newAccelerationBreakFrequency < 0.0)
            newAccelerationBreakFrequency = defaultAccelerationBreakFrequency.getDoubleValue();

         double newMaxPositionError = jointParameters.getMaxPositionError();
         if (Double.isNaN(newMaxPositionError) || newMaxPositionError < 0.0)
            newMaxPositionError = defaultIntegrationMaxPositionError.getDoubleValue();

         double newMaxAcceleration = jointParameters.getMaxAcceleration();
         if (Double.isNaN(newMaxAcceleration) || newMaxAcceleration < 0.0)
            newMaxAcceleration = defaultIntegrationMaxAcceleration.getDoubleValue();

         if (localJointIndex < 0)
         {
            jointsToComputeDesiredPositionFor.add(jointToComputeDesiredPositionFor);
            jointSpecificPositionBreakFrequency.add(newPositionBreakFrequency);
            jointSpecificAccelerationBreakFrequency.add(newAccelerationBreakFrequency);
            jointSpecificMaxPositionError.add(newMaxPositionError);
            jointSpecificMaxAcceleration.add(newMaxAcceleration);
         }
         else
         {
            jointSpecificPositionBreakFrequency.set(localJointIndex, newPositionBreakFrequency);
            jointSpecificAccelerationBreakFrequency.set(localJointIndex, newAccelerationBreakFrequency);
            jointSpecificMaxPositionError.set(localJointIndex, newMaxPositionError);
            jointSpecificMaxAcceleration.set(localJointIndex, newMaxAcceleration);
         }
      }
   }

   public void computeAndUpdateDataHolder(LowLevelOneDoFJointDesiredDataHolder lowLevelJointDataHolderToUpdate)
   {
      for (int jointIndex = 0; jointIndex < jointsToComputeDesiredPositionFor.size(); jointIndex++)
      {
         OneDoFJoint joint = jointsToComputeDesiredPositionFor.get(jointIndex);

         JointDesiredOutput lowLevelJointData = lowLevelJointDataHolderToUpdate.getJointDesiredOutput(joint);
         if (lowLevelJointData == null || !lowLevelJointData.hasDesiredVelocity())
            continue;
         if (!lowLevelJointData.hasDesiredAcceleration())
            lowLevelJointData.setDesiredAcceleration(joint.getQdd());
         if (!lowLevelJointData.hasDesiredPosition())
            lowLevelJointData.setDesiredPosition(joint.getQ());

         double desiredAcceleration = lowLevelJointData.getDesiredAcceleration();
         double desiredVelocity = lowLevelJointData.getDesiredVelocity();
         double desiredPosition = lowLevelJointData.getDesiredPosition();

         double alphaPosition = AlphaFilteredYoVariable.computeAlphaGivenBreakFrequencyProperly(jointSpecificPositionBreakFrequency.get(jointIndex), controlDT);
         double alphaAcceleration = AlphaFilteredYoVariable.computeAlphaGivenBreakFrequencyProperly(jointSpecificAccelerationBreakFrequency.get(jointIndex), controlDT);
         double maxPositionError = jointSpecificMaxPositionError.get(jointIndex);
         double maxAcceleration = jointSpecificMaxAcceleration.get(jointIndex);

         // Alpha filter the desired acceleration.
         double previousDesiredVelocity = this.previousDesiredVelocity.size() > jointIndex ? this.previousDesiredVelocity.get(jointIndex) : 0.0;
         desiredAcceleration = (desiredVelocity - previousDesiredVelocity) / controlDT * alphaAcceleration + (1.0 - alphaAcceleration) * desiredAcceleration;
         desiredAcceleration = MathTools.clamp(desiredAcceleration, maxAcceleration);

         // Decay desiredPosition towards the positionReference and then predict the desired position.
         double positionReference = joint.getQ();
         desiredPosition = desiredPosition * alphaPosition + (1.0 - alphaPosition) * positionReference;
         desiredPosition += desiredVelocity * controlDT;
         desiredPosition = MathTools.clamp(desiredPosition, positionReference - maxPositionError, positionReference + maxPositionError);

         // Limit the desired position to the joint range and recompute the desired velocity.
         desiredPosition = MathTools.clamp(desiredPosition, joint.getJointLimitLower(), joint.getJointLimitUpper());

         lowLevelJointData.setDesiredAcceleration(desiredAcceleration);
         lowLevelJointData.setDesiredPosition(desiredPosition);

         // Store the previous desired velocity into an array
         if (this.previousDesiredVelocity.size() > jointIndex)
            this.previousDesiredVelocity.set(jointIndex, desiredVelocity);
         else
            this.previousDesiredVelocity.add(desiredVelocity);
      }
   }

   public static void main(String[] args)
   {
      double controlDT = 1.0 / 250.0;
      double alpha_pos = AlphaFilteredYoVariable.computeAlphaGivenBreakFrequencyProperly(DEFAULT_POSITION_BREAK_FREQUENCY, controlDT);
      double alpha_acc = AlphaFilteredYoVariable.computeAlphaGivenBreakFrequencyProperly(DEFAULT_ACCELERATION_BREAK_FREQUENCY, controlDT);
      PrintTools.info("Alpha Position: " + alpha_pos);
      PrintTools.info("Alpha Acceleration: " + alpha_acc);
   }
}
