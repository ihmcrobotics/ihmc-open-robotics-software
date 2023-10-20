package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates;

import us.ihmc.commons.MathTools;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputBasics;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputReadOnly;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class JointControlBlender
{
   /** This is for hardware debug purposes only. */
   private static final boolean ENABLE_TAU_SCALE = false;

   private final YoDouble tauScale;
   private final OneDoFJointBasics oneDoFJoint;

   public JointControlBlender(String nameSuffix, OneDoFJointBasics oneDoFJoint, YoRegistry parentRegistry)
   {
      this.oneDoFJoint = oneDoFJoint;
      String namePrefix = oneDoFJoint.getName();

      YoRegistry registry = new YoRegistry(namePrefix + nameSuffix + "JointControlBlender");

      if (ENABLE_TAU_SCALE)
      {
         tauScale = new YoDouble("tau_scale_" + namePrefix + nameSuffix, registry);
         tauScale.set(1.0);
      }
      else
      {
         tauScale = null;
      }

      if (parentRegistry != null)
         parentRegistry.addChild(registry);
   }

   public void initialize()
   {
   }

   /**
    * Blend two joint desired outputs and pack the result in {@code outputDataToPack}.
    * <p>
    * Pseudo-code:
    * {@code outputDataToPack = (1.0 - alpha) * outputData0 + alpha * outputData1}.
    * </p>
    * 
    * @param outputDataToPack the output data in which the result is stored. Modified.
    * @param outputData0 the first output data to be blended. Not modified.
    * @param outputData1 the second output data to be blended. Not modified.
    * @param blendingFactor the blending factor.
    */
   public void computeAndUpdateJointControl(JointDesiredOutputBasics outputDataToPack, JointDesiredOutputReadOnly outputData0, JointDesiredOutputReadOnly outputData1,
                                            double blendingFactor)
   {
      blendingFactor = MathTools.clamp(blendingFactor, 0.0, 1.0);

      if (ENABLE_TAU_SCALE)
         blendingFactor *= tauScale.getDoubleValue();

      if (blendingFactor == 0.0)
      {
         outputDataToPack.set(outputData0);
         return;
      }

      if (blendingFactor == 1.0)
      {
         outputDataToPack.set(outputData1);
         return;
      }

      outputDataToPack.clear();
      outputDataToPack.setControlMode(outputData0.getControlMode());
      outputDataToPack.setLoadMode(outputData0.getLoadMode());

      if (hasDesiredPosition(outputData0) || hasDesiredPosition(outputData1))
      {
         double desiredPosition0 = hasDesiredPosition(outputData0) ? outputData0.getDesiredPosition() : oneDoFJoint.getQ();
         double desiredPosition1 = hasDesiredPosition(outputData1) ? outputData1.getDesiredPosition() : oneDoFJoint.getQ();
         double desiredPosition = EuclidCoreTools.interpolate(desiredPosition0, desiredPosition1, blendingFactor);
         outputDataToPack.setDesiredPosition(desiredPosition);
      }

      if (hasDesiredVelocity(outputData0) || hasDesiredVelocity(outputData1))
      {
         double desiredVelocity0 = hasDesiredVelocity(outputData0) ? outputData0.getDesiredVelocity() : oneDoFJoint.getQd();
         double desiredVelocity1 = hasDesiredVelocity(outputData1) ? outputData1.getDesiredVelocity() : oneDoFJoint.getQd();
         double desiredVelocity = EuclidCoreTools.interpolate(desiredVelocity0, desiredVelocity1, blendingFactor);
         outputDataToPack.setDesiredVelocity(desiredVelocity);
      }

      if (hasDesiredAcceleration(outputData0) || hasDesiredAcceleration(outputData1))
      {
         double desiredAcceleration0 = hasDesiredAcceleration(outputData0) ? outputData0.getDesiredAcceleration() : 0.0;
         double desiredAcceleration1 = hasDesiredAcceleration(outputData1) ? outputData1.getDesiredAcceleration() : 0.0;
         double desiredAcceleration = EuclidCoreTools.interpolate(desiredAcceleration0, desiredAcceleration1, blendingFactor);
         outputDataToPack.setDesiredAcceleration(desiredAcceleration);
      }

      if (hasDesiredTorque(outputData0) || hasDesiredTorque(outputData1))
      {
         double desiredTorque0 = hasDesiredTorque(outputData0) ? outputData0.getDesiredTorque() : 0.0;
         double desiredTorque1 = hasDesiredTorque(outputData1) ? outputData1.getDesiredTorque() : 0.0;
         double desiredTorque = EuclidCoreTools.interpolate(desiredTorque0, desiredTorque1, blendingFactor);
         outputDataToPack.setDesiredTorque(desiredTorque);
      }

      if (hasPositionFeedbackMaxError(outputData0) || hasPositionFeedbackMaxError(outputData1))
      {
         double maxError0 = hasPositionFeedbackMaxError(outputData0) ? outputData0.getPositionFeedbackMaxError() : 0.0;
         double maxError1 = hasPositionFeedbackMaxError(outputData1) ? outputData1.getPositionFeedbackMaxError() : 0.0;
         double maxError = EuclidCoreTools.interpolate(maxError0, maxError1, blendingFactor);
         outputDataToPack.setPositionFeedbackMaxError(maxError);
      }

      if (hasVelocityFeedbackMaxError(outputData0) || hasVelocityFeedbackMaxError(outputData1))
      {
         double maxError0 = hasVelocityFeedbackMaxError(outputData0) ? outputData0.getVelocityFeedbackMaxError() : 0.0;
         double maxError1 = hasVelocityFeedbackMaxError(outputData1) ? outputData1.getVelocityFeedbackMaxError() : 0.0;
         double maxError = EuclidCoreTools.interpolate(maxError0, maxError1, blendingFactor);
         outputDataToPack.setVelocityFeedbackMaxError(maxError);
      }

      if (hasStiffness(outputData0) || hasStiffness(outputData1))
      {
         double stiffness0 = hasStiffness(outputData0) ? outputData0.getStiffness() : 0.0;
         double stiffness1 = hasStiffness(outputData1) ? outputData1.getStiffness() : 0.0;
         double stiffness = EuclidCoreTools.interpolate(stiffness0, stiffness1, blendingFactor);
         outputDataToPack.setStiffness(stiffness);
      }

      if (hasDamping(outputData0) || hasDamping(outputData1))
      {
         double damping0 = hasDamping(outputData0) ? outputData0.getDamping() : 0.0;
         double damping1 = hasDamping(outputData1) ? outputData1.getDamping() : 0.0;
         double damping = EuclidCoreTools.interpolate(damping0, damping1, blendingFactor);
         outputDataToPack.setDamping(damping);
      }

      if (hasVelocityScaling(outputData0) || hasVelocityScaling(outputData1))
      {
         double velocityScaling0 = hasVelocityScaling(outputData0) ? outputData0.getVelocityScaling() : 0.0;
         double velocityScaling1 = hasVelocityScaling(outputData1) ? outputData1.getVelocityScaling() : 0.0;
         double velocityScaling = EuclidCoreTools.interpolate(velocityScaling0, velocityScaling1, blendingFactor);
         outputDataToPack.setVelocityScaling(velocityScaling);
      }

      if (hasVelocityIntegrationMaxError(outputData0) || hasVelocityIntegrationMaxError(outputData1))
      {
         double maxError0 = hasVelocityIntegrationMaxError(outputData0) ? outputData0.getVelocityIntegrationMaxError() : 0.0;
         double maxError1 = hasVelocityIntegrationMaxError(outputData1) ? outputData1.getVelocityIntegrationMaxError() : 0.0;
         double maxError = EuclidCoreTools.interpolate(maxError0, maxError1, blendingFactor);
         outputDataToPack.setVelocityIntegrationMaxError(maxError);
      }

      if (hasVelocityIntegrationBreakFrequency(outputData0) || hasVelocityIntegrationBreakFrequency(outputData1))
      {
         double frequency0 = hasVelocityIntegrationBreakFrequency(outputData0) ? outputData0.getVelocityIntegrationBreakFrequency() : 0.0;
         double frequency1 = hasVelocityIntegrationBreakFrequency(outputData1) ? outputData1.getVelocityIntegrationBreakFrequency() : 0.0;
         double frequency = EuclidCoreTools.interpolate(frequency0, frequency1, blendingFactor);
         outputDataToPack.setVelocityIntegrationBreakFrequency(frequency);
      }

      if (hasPositionIntegrationMaxError(outputData0) || hasPositionIntegrationMaxError(outputData1))
      {
         double maxError0 = hasPositionIntegrationMaxError(outputData0) ? outputData0.getPositionIntegrationMaxError() : 0.0;
         double maxError1 = hasPositionIntegrationMaxError(outputData1) ? outputData1.getPositionIntegrationMaxError() : 0.0;
         double maxError = EuclidCoreTools.interpolate(maxError0, maxError1, blendingFactor);
         outputDataToPack.setPositionIntegrationMaxError(maxError);
      }

      if (hasPositionIntegrationBreakFrequency(outputData0) || hasPositionIntegrationBreakFrequency(outputData1))
      {
         double frequency0 = hasPositionIntegrationBreakFrequency(outputData0) ? outputData0.getPositionIntegrationBreakFrequency() : 0.0;
         double frequency1 = hasPositionIntegrationBreakFrequency(outputData1) ? outputData1.getPositionIntegrationBreakFrequency() : 0.0;
         double frequency = EuclidCoreTools.interpolate(frequency0, frequency1, blendingFactor);
         outputDataToPack.setPositionIntegrationBreakFrequency(frequency);
      }

      outputDataToPack.setResetIntegrators(pollResetIntegratorsRequest(outputData0) || pollResetIntegratorsRequest(outputData1));
   }

   private boolean hasDesiredPosition(JointDesiredOutputReadOnly outputData)
   {
      return outputData != null && outputData.hasDesiredPosition();
   }

   private boolean hasDesiredVelocity(JointDesiredOutputReadOnly outputData)
   {
      return outputData != null && outputData.hasDesiredVelocity();
   }

   private boolean hasDesiredAcceleration(JointDesiredOutputReadOnly outputData)
   {
      return outputData != null && outputData.hasDesiredAcceleration();
   }

   private boolean hasDesiredTorque(JointDesiredOutputReadOnly outputData)
   {
      return outputData != null && outputData.hasDesiredTorque();
   }

   private boolean hasPositionFeedbackMaxError(JointDesiredOutputReadOnly outputData)
   {
      return outputData != null && outputData.hasPositionFeedbackMaxError();
   }

   private boolean hasVelocityFeedbackMaxError(JointDesiredOutputReadOnly outputData)
   {
      return outputData != null && outputData.hasVelocityFeedbackMaxError();
   }

   private boolean hasStiffness(JointDesiredOutputReadOnly outputData)
   {
      return outputData != null && outputData.hasStiffness();
   }

   private boolean hasDamping(JointDesiredOutputReadOnly outputData)
   {
      return outputData != null && outputData.hasDamping();
   }

   private boolean hasVelocityScaling(JointDesiredOutputReadOnly outputData)
   {
      return outputData != null && outputData.hasVelocityScaling();
   }

   private boolean hasVelocityIntegrationBreakFrequency(JointDesiredOutputReadOnly outputData)
   {
      return outputData != null && outputData.hasVelocityIntegrationBreakFrequency();
   }

   private boolean hasVelocityIntegrationMaxError(JointDesiredOutputReadOnly outputData)
   {
      return outputData != null && outputData.hasVelocityIntegrationMaxError();
   }

   private boolean hasPositionIntegrationBreakFrequency(JointDesiredOutputReadOnly outputData)
   {
      return outputData != null && outputData.hasPositionIntegrationBreakFrequency();
   }

   private boolean hasPositionIntegrationMaxError(JointDesiredOutputReadOnly outputData)
   {
      return outputData != null && outputData.hasPositionIntegrationMaxError();
   }

   private boolean pollResetIntegratorsRequest(JointDesiredOutputReadOnly outputData)
   {
      return outputData != null && outputData.pollResetIntegratorsRequest();
   }
}
