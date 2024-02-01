package us.ihmc.commonWalkingControlModules.momentumBasedController.optimization;

import java.util.List;

import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.JointAccelerationIntegrationCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.lowLevel.LowLevelOneDoFJointDesiredDataHolder;
import us.ihmc.commonWalkingControlModules.controllerCore.parameters.JointAccelerationIntegrationParametersReadOnly;
import us.ihmc.commonWalkingControlModules.controllerCore.parameters.JointVelocityIntegratorResetMode;
import us.ihmc.commons.MathTools;
import us.ihmc.log.LogTools;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.multiBodySystem.iterators.SubtreeStreams;
import us.ihmc.robotics.math.filters.AlphaFilteredYoVariable;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputBasics;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputListBasics;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoEnum;

public class JointAccelerationIntegrationCalculator
{
   public static final double DEFAULT_POSITION_BREAK_FREQUENCY = 0.016;
   public static final double DEFAULT_VELOCITY_BREAK_FREQUENCY = 2.04;
   public static final double DEFAULT_MAX_POSITION_ERROR = 0.2;
   public static final double DEFAULT_MAX_VELOCITY_ERROR = 2.0;
   public static final double DEFAULT_VELOCITY_REFERENCE_ALPHA = 0.0;
   public static final JointVelocityIntegratorResetMode DEFAULT_VELOCITY_RESET_MODE = JointVelocityIntegratorResetMode.CURRENT_VELOCITY;

   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());

   private final List<OneDoFJointBasics> joints;
   private final List<JointParameters> jointParametersList;

   private final YoDouble defaultPositionBreakFrequency = new YoDouble("defaultPositionBreakFrequencyIntegration", registry);
   private final YoDouble defaultVelocityBreakFrequency = new YoDouble("defaultVelocityBreakFrequencyIntegration", registry);
   private final YoDouble defaultIntegrationMaxVelocityError = new YoDouble("defaultIntegrationMaxVelocityError", registry);
   private final YoDouble defaultIntegrationMaxPositionError = new YoDouble("defaultIntegrationMaxPositionError", registry);
   private final YoDouble defaultVelocityReferenceAlpha = new YoDouble("defaultVelocityReferenceAlpha", registry);
   private final YoEnum<JointVelocityIntegratorResetMode> defaultVelocityResetMode = new YoEnum<>("defaultVelocityResetMode",
                                                                                                  registry,
                                                                                                  JointVelocityIntegratorResetMode.class);

   private final double controlDT;

   public JointAccelerationIntegrationCalculator(RigidBodyBasics rootBody, double controlDT, YoRegistry parentRegistry)
   {
      this.controlDT = controlDT;
      defaultPositionBreakFrequency.set(DEFAULT_POSITION_BREAK_FREQUENCY);
      defaultVelocityBreakFrequency.set(DEFAULT_VELOCITY_BREAK_FREQUENCY);
      defaultIntegrationMaxPositionError.set(DEFAULT_MAX_POSITION_ERROR);
      defaultIntegrationMaxVelocityError.set(DEFAULT_MAX_VELOCITY_ERROR);
      defaultVelocityReferenceAlpha.set(DEFAULT_VELOCITY_REFERENCE_ALPHA);
      defaultVelocityResetMode.set(DEFAULT_VELOCITY_RESET_MODE);

      joints = SubtreeStreams.fromChildren(OneDoFJointBasics.class, rootBody).toList();
      jointParametersList = joints.stream().map(j -> new JointParameters()).toList();

      parentRegistry.addChild(registry);
   }

   public void resetJointParameters()
   {
      for (int i = 0; i < joints.size(); i++)
         jointParametersList.get(i).clear();
   }

   public void submitJointAccelerationIntegrationCommand(JointAccelerationIntegrationCommand command)
   {
      for (int commandJointIndex = 0; commandJointIndex < command.getNumberOfJointsToComputeDesiredPositionFor(); commandJointIndex++)
      {
         OneDoFJointBasics jointToComputeDesierdPositionFor = command.getJointToComputeDesiredPositionFor(commandJointIndex);
         int localJointIndex = joints.indexOf(jointToComputeDesierdPositionFor);
         jointParametersList.get(localJointIndex).complete(command.getJointParameters(commandJointIndex));
      }
   }

   public void computeAndUpdateDataHolder(JointDesiredOutputListBasics lowLevelJointDataHolderToUpdate)
   {
      for (int jointIndex = 0; jointIndex < joints.size(); jointIndex++)
      {
         OneDoFJointBasics joint = joints.get(jointIndex);
         JointParameters parameters = jointParametersList.get(jointIndex);

         JointDesiredOutputBasics lowLevelJointData = lowLevelJointDataHolderToUpdate.getJointDesiredOutput(joint);
         if (lowLevelJointData == null || !lowLevelJointData.hasDesiredAcceleration() || !parameters.isEnabled())
            continue;

         double alphaPosition = AlphaFilteredYoVariable.computeAlphaGivenBreakFrequencyProperly(parameters.getPositionBreakFrequency(), controlDT);
         double alphaVelocity = AlphaFilteredYoVariable.computeAlphaGivenBreakFrequencyProperly(parameters.getVelocityBreakFrequency(), controlDT);
         double maxPositionError = parameters.getMaxPositionError();
         double maxVelocityError = parameters.getMaxVelocityError();
         double velocityReferenceAlpha = parameters.getVelocityReferenceAlpha();

         double velocityReference = velocityReferenceAlpha * joint.getQd();
         double positionReference = joint.getQ();

         boolean resetIntegrators = lowLevelJointData.pollResetIntegratorsRequest();
         if (!lowLevelJointData.hasDesiredVelocity() || resetIntegrators)
         {
            JointVelocityIntegratorResetMode velocityResetMode = parameters.getVelocityResetMode();
            switch (velocityResetMode)
            {
               case CURRENT_VELOCITY:
                  lowLevelJointData.setDesiredVelocity(joint.getQd());
                  break;
               case ZERO_VELOCITY:
                  lowLevelJointData.setDesiredVelocity(0.0);
                  break;
               case REFERENCE_VELOCITY:
                  lowLevelJointData.setDesiredVelocity(velocityReference);
                  break;
               default:
                  LogTools.warn("Unexpected velocity reset mode: {}, resetting velocity to current for joint {}.", velocityResetMode, joint.getName());
                  lowLevelJointData.setDesiredVelocity(joint.getQd());
                  break;
            }
         }

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

   public static void main(String[] args)
   {
      System.out.println(AlphaFilteredYoVariable.computeAlphaGivenBreakFrequencyProperly(0.016, 3e-3));
      System.out.println(AlphaFilteredYoVariable.computeAlphaGivenBreakFrequencyProperly(2.7, 3e-3));
   }

   private enum JointIntegratorState
   {
      ENABLED, DISABLED
   };

   private class JointParameters
   {
      private JointIntegratorState integratorState;
      private double positionBreakFrequency;
      private double velocityBreakFrequency;
      private double maxPositionError;
      private double maxVelocityError;
      private double velocityReferenceAlpha;
      private JointVelocityIntegratorResetMode velocityResetMode;

      public JointParameters()
      {
         clear();
      }

      private void clear()
      {
         integratorState = null;
         positionBreakFrequency = Double.NaN;
         velocityBreakFrequency = Double.NaN;
         maxPositionError = Double.NaN;
         maxVelocityError = Double.NaN;
         velocityReferenceAlpha = Double.NaN;
         velocityResetMode = null;
      }

      private void complete(JointAccelerationIntegrationParametersReadOnly cmd)
      {
         if (cmd.getDisableAccelerationIntegration() || integratorState == null)
            integratorState = cmd.getDisableAccelerationIntegration() ? JointIntegratorState.DISABLED : JointIntegratorState.ENABLED;

         if (!Double.isNaN(cmd.getPositionBreakFrequency()) && cmd.getPositionBreakFrequency() >= 0.0)
            positionBreakFrequency = cmd.getPositionBreakFrequency();

         if (!Double.isNaN(cmd.getVelocityBreakFrequency()) && cmd.getVelocityBreakFrequency() >= 0.0)
            velocityBreakFrequency = cmd.getVelocityBreakFrequency();

         if (!Double.isNaN(cmd.getMaxPositionError()) && cmd.getMaxPositionError() >= 0.0)
            maxPositionError = cmd.getMaxPositionError();

         if (!Double.isNaN(cmd.getMaxVelocityError()) && cmd.getMaxVelocityError() >= 0.0)
            maxVelocityError = cmd.getMaxVelocityError();

         if (!Double.isNaN(cmd.getVelocityReferenceAlpha()) && cmd.getVelocityReferenceAlpha() >= 0.0)
            velocityReferenceAlpha = MathTools.clamp(cmd.getVelocityReferenceAlpha(), 0.0, 1.0);

         if (cmd.getVelocityResetMode() != null)
            velocityResetMode = cmd.getVelocityResetMode();
      }

      public boolean isEnabled()
      {
         // null is considered disabled as it means no command was submitted for that joint.
         return integratorState == JointIntegratorState.ENABLED;
      }

      public double getPositionBreakFrequency()
      {
         if (Double.isNaN(positionBreakFrequency))
            return defaultPositionBreakFrequency.getValue();
         else
            return positionBreakFrequency;
      }

      public double getVelocityBreakFrequency()
      {
         if (Double.isNaN(velocityBreakFrequency))
            return defaultVelocityBreakFrequency.getValue();
         else
            return velocityBreakFrequency;
      }

      public double getMaxPositionError()
      {
         if (Double.isNaN(maxPositionError))
            return defaultIntegrationMaxPositionError.getValue();
         else
            return maxPositionError;
      }

      public double getMaxVelocityError()
      {
         if (Double.isNaN(maxVelocityError))
            return defaultIntegrationMaxVelocityError.getValue();
         else
            return maxVelocityError;
      }

      public double getVelocityReferenceAlpha()
      {
         if (Double.isNaN(velocityReferenceAlpha))
            return defaultVelocityReferenceAlpha.getValue();
         else
            return velocityReferenceAlpha;
      }

      public JointVelocityIntegratorResetMode getVelocityResetMode()
      {
         if (velocityResetMode == null)
            return defaultVelocityResetMode.getValue();
         else
            return velocityResetMode;
      }
   }
}
