package us.ihmc.commonWalkingControlModules.momentumBasedController.optimization;

import java.util.ArrayList;
import java.util.List;

import gnu.trove.list.array.TDoubleArrayList;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.JointAccelerationIntegrationCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.lowLevel.LowLevelOneDoFJointDesiredDataHolder;
import us.ihmc.commonWalkingControlModules.controllerCore.parameters.JointAccelerationIntegrationParametersReadOnly;
import us.ihmc.commons.MathTools;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.robotics.math.filters.AlphaFilteredYoVariable;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputBasics;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class JointAccelerationIntegrationCalculator
{
   public static final double DEFAULT_POSITION_BREAK_FREQUENCY = 0.016;
   public static final double DEFAULT_VELOCITY_BREAK_FREQUENCY = 2.04;
   public static final double DEFAULT_MAX_POSITION_ERROR = 0.2;
   public static final double DEFAULT_MAX_VELOCITY_ERROR = 2.0;
   public static final double DEFAULT_VELOCITY_REFERENCE_ALPHA = 0.0;

   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());

   private final List<OneDoFJointBasics> jointsToComputeDesiredPositionFor = new ArrayList<>();
   private final TDoubleArrayList jointSpecificPositionBreakFrequency = new TDoubleArrayList();
   private final TDoubleArrayList jointSpecificVelocityBreakFrequency = new TDoubleArrayList();
   private final TDoubleArrayList jointSpecificMaxPositionError = new TDoubleArrayList();
   private final TDoubleArrayList jointSpecificMaxVelocityError = new TDoubleArrayList();
   private final TDoubleArrayList jointSpecificVelocityReferenceAlpha = new TDoubleArrayList();

   private final YoDouble defaultPositionBreakFrequency = new YoDouble("defaultPositionBreakFrequencyIntegration", registry);
   private final YoDouble defaultVelocityBreakFrequency = new YoDouble("defaultVelocityBreakFrequencyIntegration", registry);
   private final YoDouble defaultIntegrationMaxVelocityError = new YoDouble("defaultIntegrationMaxVelocityError", registry);
   private final YoDouble defaultIntegrationMaxPositionError = new YoDouble("defaultIntegrationMaxPositionError", registry);
   private final YoDouble defaultVelocityReferenceAlpha = new YoDouble("defaultVelocityReferenceAlpha", registry);

   private final double controlDT;

   public JointAccelerationIntegrationCalculator(double controlDT, YoRegistry parentRegistry)
   {
      this.controlDT = controlDT;
      defaultPositionBreakFrequency.set(DEFAULT_POSITION_BREAK_FREQUENCY);
      defaultVelocityBreakFrequency.set(DEFAULT_VELOCITY_BREAK_FREQUENCY);
      defaultIntegrationMaxPositionError.set(DEFAULT_MAX_POSITION_ERROR);
      defaultIntegrationMaxVelocityError.set(DEFAULT_MAX_VELOCITY_ERROR);
      defaultVelocityReferenceAlpha.set(DEFAULT_VELOCITY_REFERENCE_ALPHA);

      parentRegistry.addChild(registry);
   }

   public void submitJointAccelerationIntegrationCommand(JointAccelerationIntegrationCommand command)
   {
      for (int commandJointIndex = 0; commandJointIndex < command.getNumberOfJointsToComputeDesiredPositionFor(); commandJointIndex++)
      {
         OneDoFJointBasics jointToComputeDesierdPositionFor = command.getJointToComputeDesiredPositionFor(commandJointIndex);
         int localJointIndex = jointsToComputeDesiredPositionFor.indexOf(jointToComputeDesierdPositionFor);
         JointAccelerationIntegrationParametersReadOnly jointParameters = command.getJointParameters(commandJointIndex);

         double newPositionBreakFrequency = jointParameters.getPositionBreakFrequency();
         if (Double.isNaN(newPositionBreakFrequency) || newPositionBreakFrequency < 0.0)
            newPositionBreakFrequency = defaultPositionBreakFrequency.getDoubleValue();
         newPositionBreakFrequency = Math.max(0.0, newPositionBreakFrequency);

         double newVelocityBreakFrequency = jointParameters.getVelocityBreakFrequency();
         if (Double.isNaN(newVelocityBreakFrequency) || newVelocityBreakFrequency < 0.0)
            newVelocityBreakFrequency = defaultVelocityBreakFrequency.getDoubleValue();
         newVelocityBreakFrequency = Math.max(0.0, newVelocityBreakFrequency);

         double newMaxPositionError = jointParameters.getMaxPositionError();
         if (Double.isNaN(newMaxPositionError) || newMaxPositionError < 0.0)
            newMaxPositionError = defaultIntegrationMaxPositionError.getDoubleValue();
         newMaxPositionError = Math.max(0.0, newMaxPositionError);

         double newMaxVelocityError = jointParameters.getMaxVelocityError();
         if (Double.isNaN(newMaxVelocityError) || newMaxVelocityError < 0.0)
            newMaxVelocityError = defaultIntegrationMaxVelocityError.getDoubleValue();
         newMaxVelocityError = Math.max(0.0, newMaxVelocityError);

         double newVelocityReferenceAlpha = jointParameters.getVelocityReferenceAlpha();
         if (Double.isNaN(newVelocityReferenceAlpha) || newVelocityReferenceAlpha < 0.0)
            newVelocityReferenceAlpha = defaultVelocityReferenceAlpha.getDoubleValue();
         newVelocityReferenceAlpha = MathTools.clamp(newVelocityReferenceAlpha, 0.0, 1.0);

         if (localJointIndex < 0)
         {
            jointsToComputeDesiredPositionFor.add(jointToComputeDesierdPositionFor);
            jointSpecificPositionBreakFrequency.add(newPositionBreakFrequency);
            jointSpecificVelocityBreakFrequency.add(newVelocityBreakFrequency);
            jointSpecificMaxPositionError.add(newMaxPositionError);
            jointSpecificMaxVelocityError.add(newMaxVelocityError);
            jointSpecificVelocityReferenceAlpha.add(newVelocityReferenceAlpha);
         }
         else
         {
            jointSpecificPositionBreakFrequency.set(localJointIndex, newPositionBreakFrequency);
            jointSpecificVelocityBreakFrequency.set(localJointIndex, newVelocityBreakFrequency);
            jointSpecificMaxPositionError.set(localJointIndex, newMaxPositionError);
            jointSpecificMaxVelocityError.set(localJointIndex, newMaxVelocityError);
            jointSpecificVelocityReferenceAlpha.set(localJointIndex, newVelocityReferenceAlpha);
         }
      }
   }

   public void computeAndUpdateDataHolder(LowLevelOneDoFJointDesiredDataHolder lowLevelJointDataHolderToUpdate)
   {
      for (int jointIndex = 0; jointIndex < jointsToComputeDesiredPositionFor.size(); jointIndex++)
      {
         OneDoFJointBasics joint = jointsToComputeDesiredPositionFor.get(jointIndex);

         JointDesiredOutputBasics lowLevelJointData = lowLevelJointDataHolderToUpdate.getJointDesiredOutput(joint);
         if (lowLevelJointData == null || !lowLevelJointData.hasDesiredAcceleration())
            continue;

         double alphaPosition = AlphaFilteredYoVariable.computeAlphaGivenBreakFrequencyProperly(jointSpecificPositionBreakFrequency.get(jointIndex), controlDT);
         double alphaVelocity = AlphaFilteredYoVariable.computeAlphaGivenBreakFrequencyProperly(jointSpecificVelocityBreakFrequency.get(jointIndex), controlDT);
         double maxPositionError = jointSpecificMaxPositionError.get(jointIndex);
         double maxVelocityError = jointSpecificMaxVelocityError.get(jointIndex);
         double velocityReferenceAlpha = jointSpecificVelocityReferenceAlpha.get(jointIndex);

         double velocityReference = velocityReferenceAlpha * joint.getQd();
         double positionReference = joint.getQ();

         boolean resetIntegrators = lowLevelJointData.pollResetIntegratorsRequest();
         if (!lowLevelJointData.hasDesiredVelocity() || resetIntegrators)
            lowLevelJointData.setDesiredVelocity(velocityReference);
         if (!lowLevelJointData.hasDesiredPosition() || resetIntegrators)
            lowLevelJointData.setDesiredPosition(positionReference);

         double desiredAcceleration = lowLevelJointData.getDesiredAcceleration();
         double desiredVelocity = lowLevelJointData.getDesiredVelocity();
         double desiredPosition = lowLevelJointData.getDesiredPosition();

         // Decay desiredVelocity towards the velocityReference and then predict the desired velocity.
         desiredVelocity = desiredVelocity * alphaVelocity + (1.0 - alphaVelocity) * velocityReference;
         desiredVelocity += desiredAcceleration * controlDT;
         desiredVelocity = MathTools.clamp(desiredVelocity, velocityReference - maxVelocityError, velocityReference + maxVelocityError);

         // Decay desiredPosition towards the positionReference and then predict the desired position.
         desiredPosition = desiredPosition * alphaPosition + (1.0 - alphaPosition) * positionReference;
         desiredPosition += desiredVelocity * controlDT;
         desiredPosition = MathTools.clamp(desiredPosition, positionReference - maxPositionError, positionReference + maxPositionError);

         // Limit the desired position to the joint range and recompute the desired velocity.
         desiredPosition = MathTools.clamp(desiredPosition, joint.getJointLimitLower(), joint.getJointLimitUpper());

         // June 20, 2018: Removed this as is seems to cause instability.
         //         desiredVelocity = (desiredPosition - lowLevelJointData.getDesiredPosition()) / controlDT;

         lowLevelJointData.setDesiredVelocity(desiredVelocity);
         lowLevelJointData.setDesiredPosition(desiredPosition);
      }
   }
}
