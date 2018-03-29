package us.ihmc.commonWalkingControlModules.momentumBasedController.optimization;

import java.util.ArrayList;
import java.util.List;

import gnu.trove.list.array.TDoubleArrayList;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.JointAccelerationIntegrationCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.lowLevel.LowLevelOneDoFJointDesiredDataHolder;
import us.ihmc.commonWalkingControlModules.controllerCore.parameters.JointAccelerationIntegrationParametersReadOnly;
import us.ihmc.commons.MathTools;
import us.ihmc.commons.PrintTools;
import us.ihmc.robotics.math.filters.AlphaFilteredYoVariable;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutput;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class JointAccelerationIntegrationCalculator
{
   public static final double DEFAULT_POSITION_BREAK_FREQUENCY = 0.016;
   public static final double DEFAULT_VELOCITY_BREAK_FREQUENCY = 2.04;
   public static final double DEFAULT_MAX_POSITION_ERROR = 0.2;
   public static final double DEFAULT_MAX_VELOCITY = 2.0;

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final List<OneDoFJoint> jointsToComputeDesiredPositionFor = new ArrayList<>();
   private final TDoubleArrayList jointSpecificPositionBreakFrequency = new TDoubleArrayList();
   private final TDoubleArrayList jointSpecificVelocityBreakFrequency = new TDoubleArrayList();
   private final TDoubleArrayList jointSpecificMaxPositionError = new TDoubleArrayList();
   private final TDoubleArrayList jointSpecificMaxVelocity = new TDoubleArrayList();

   private final YoDouble defaultPositionBreakFrequency = new YoDouble("defaultPositionBreakFrequencyIntegration", registry);
   private final YoDouble defaultVelocityBreakFrequency = new YoDouble("defaultVelocityBreakFrequencyIntegration", registry);
   private final YoDouble defaultIntegrationMaxVelocity = new YoDouble("defaultIntegrationMaxVelocity", registry);
   private final YoDouble defaultIntegrationMaxPositionError = new YoDouble("defaultIntegrationMaxPositionError", registry);

   private final double controlDT;

   public JointAccelerationIntegrationCalculator(double controlDT, YoVariableRegistry parentRegistry)
   {
      this.controlDT = controlDT;
      defaultPositionBreakFrequency.set(DEFAULT_POSITION_BREAK_FREQUENCY);
      defaultVelocityBreakFrequency.set(DEFAULT_VELOCITY_BREAK_FREQUENCY);
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

         double newPositionBreakFrequency = jointParameters.getPositionBreakFrequency();
         if (Double.isNaN(newPositionBreakFrequency) || newPositionBreakFrequency < 0.0)
            newPositionBreakFrequency = defaultPositionBreakFrequency.getDoubleValue();

         double newVelocityBreakFrequency = jointParameters.getVelocityBreakFrequency();
         if (Double.isNaN(newVelocityBreakFrequency) || newVelocityBreakFrequency < 0.0)
            newVelocityBreakFrequency = defaultVelocityBreakFrequency.getDoubleValue();

         double newMaxPositionError = jointParameters.getMaxPositionError();
         if (Double.isNaN(newMaxPositionError) || newMaxPositionError < 0.0)
            newMaxPositionError = defaultIntegrationMaxPositionError.getDoubleValue();

         double newMaxVelocity = jointParameters.getMaxVelocity();
         if (Double.isNaN(newMaxVelocity) || newMaxVelocity < 0.0)
            newMaxVelocity = defaultIntegrationMaxVelocity.getDoubleValue();

         if (localJointIndex < 0)
         {
            jointsToComputeDesiredPositionFor.add(jointToComputeDesierdPositionFor);
            jointSpecificPositionBreakFrequency.add(newPositionBreakFrequency);
            jointSpecificVelocityBreakFrequency.add(newVelocityBreakFrequency);
            jointSpecificMaxPositionError.add(newMaxPositionError);
            jointSpecificMaxVelocity.add(newMaxVelocity);
         }
         else
         {
            jointSpecificPositionBreakFrequency.set(localJointIndex, newPositionBreakFrequency);
            jointSpecificVelocityBreakFrequency.set(localJointIndex, newVelocityBreakFrequency);
            jointSpecificMaxPositionError.set(localJointIndex, newMaxPositionError);
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

         double alphaPosition = AlphaFilteredYoVariable.computeAlphaGivenBreakFrequencyProperly(jointSpecificPositionBreakFrequency.get(jointIndex), controlDT);
         double alphaVelocity = AlphaFilteredYoVariable.computeAlphaGivenBreakFrequencyProperly(jointSpecificVelocityBreakFrequency.get(jointIndex), controlDT);
         double maxPositionError = jointSpecificMaxPositionError.get(jointIndex);
         double maxVelocity = jointSpecificMaxVelocity.get(jointIndex);

         // Decay desiredVelocity towards the velocityReference and then predict the desired velocity.
         double velocityReference = 0.0;
         desiredVelocity = desiredVelocity * alphaVelocity + (1.0 - alphaVelocity) * velocityReference;
         desiredVelocity += desiredAcceleration * controlDT;
         desiredVelocity = MathTools.clamp(desiredVelocity, velocityReference - maxVelocity, velocityReference + maxVelocity);

         // Decay desiredPosition towards the positionReference and then predict the desired position.
         double positionReference = joint.getQ();
         desiredPosition = desiredPosition * alphaPosition + (1.0 - alphaPosition) * positionReference;
         desiredPosition += desiredVelocity * controlDT;
         desiredPosition = MathTools.clamp(desiredPosition, positionReference - maxPositionError, positionReference + maxPositionError);

         // Limit the desired position to the joint range and recompute the desired velocity.
         desiredPosition = MathTools.clamp(desiredPosition, joint.getJointLimitLower(), joint.getJointLimitUpper());
         desiredVelocity = (desiredPosition - lowLevelJointData.getDesiredPosition()) / controlDT;

         lowLevelJointData.setDesiredVelocity(desiredVelocity);
         lowLevelJointData.setDesiredPosition(desiredPosition);
      }
   }

   public static void main(String[] args)
   {
      double controlDT = 1.0 / 250.0;
      double alpha_pos = AlphaFilteredYoVariable.computeAlphaGivenBreakFrequencyProperly(DEFAULT_POSITION_BREAK_FREQUENCY, controlDT);
      double alpha_vel = AlphaFilteredYoVariable.computeAlphaGivenBreakFrequencyProperly(DEFAULT_VELOCITY_BREAK_FREQUENCY, controlDT);
      PrintTools.info("Alpha Position: " + alpha_pos);
      PrintTools.info("Alpha Velocity: " + alpha_vel);
   }
}
