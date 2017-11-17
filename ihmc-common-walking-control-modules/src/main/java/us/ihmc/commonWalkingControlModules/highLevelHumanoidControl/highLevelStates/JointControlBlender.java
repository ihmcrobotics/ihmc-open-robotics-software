package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates;

import us.ihmc.commons.MathTools;
import us.ihmc.euclid.tools.TupleTools;
import us.ihmc.robotics.math.filters.DeltaLimitedYoVariable;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.sensorProcessing.outputData.JointDesiredControlMode;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutput;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputReadOnly;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class JointControlBlender
{
   /** This is for hardware debug purposes only. */
   private static final boolean ENABLE_TAU_SCALE = false;

   private final YoDouble tauScale;
   private final DeltaLimitedYoVariable positionStepSizeLimiter;
   private final DeltaLimitedYoVariable velocityStepSizeLimiter;

   private final OneDoFJoint oneDoFJoint;

   public JointControlBlender(String nameSuffix, OneDoFJoint oneDoFJoint, YoVariableRegistry parentRegistry)
   {
      this.oneDoFJoint = oneDoFJoint;
      String namePrefix = oneDoFJoint.getName();

      YoVariableRegistry registry = new YoVariableRegistry(namePrefix + nameSuffix + "JointControlBlender");

      this.positionStepSizeLimiter = new DeltaLimitedYoVariable(namePrefix + "PositionStepSizeLimiter", registry, 0.15);
      this.velocityStepSizeLimiter = new DeltaLimitedYoVariable(namePrefix + "VelocityStepSizeLimiter", registry, 1.5);

      if (ENABLE_TAU_SCALE)
      {
         tauScale = new YoDouble("tau_scale_" + namePrefix + nameSuffix, registry);
         tauScale.set(1.0);
      }
      else
      {
         tauScale = null;
      }

      parentRegistry.addChild(registry);
   }

   public void initialize()
   {
      double q = oneDoFJoint.getQ();
      double qd = oneDoFJoint.getQd();
      positionStepSizeLimiter.updateOutput(q, q);
      velocityStepSizeLimiter.updateOutput(qd, qd);
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
   public void computeAndUpdateJointControl(JointDesiredOutput outputDataToPack, JointDesiredOutputReadOnly outputData0, JointDesiredOutputReadOnly outputData1,
                                            double blendingFactor)
   {
      blendingFactor = MathTools.clamp(blendingFactor, 0.0, 1.0);

      if (ENABLE_TAU_SCALE)
         blendingFactor *= tauScale.getDoubleValue();

      JointDesiredControlMode controlMode = outputDataToPack.getControlMode();
      outputDataToPack.clear();
      outputDataToPack.setControlMode(controlMode);

      double currentJointAngle = oneDoFJoint.getQ();
      double currentJointVelocity = oneDoFJoint.getQd();

      if (hasDesiredPosition(outputData0) || hasDesiredPosition(outputData1))
      {
         double desiredPosition0 = hasDesiredPosition(outputData0) ? outputData0.getDesiredPosition() : oneDoFJoint.getQ();
         double desiredPosition1 = hasDesiredPosition(outputData1) ? outputData1.getDesiredPosition() : oneDoFJoint.getQ();
         double desiredPosition = TupleTools.interpolate(desiredPosition0, desiredPosition1, blendingFactor);
         positionStepSizeLimiter.updateOutput(currentJointAngle, desiredPosition);
         outputDataToPack.setDesiredPosition(positionStepSizeLimiter.getDoubleValue());
      }

      if (hasDesiredVelocity(outputData0) || hasDesiredVelocity(outputData1))
      {
         double desiredVelocity0 = hasDesiredVelocity(outputData0) ? outputData0.getDesiredVelocity() : oneDoFJoint.getQd();
         double desiredVelocity1 = hasDesiredVelocity(outputData1) ? outputData1.getDesiredVelocity() : oneDoFJoint.getQd();
         double desiredVelocity = TupleTools.interpolate(desiredVelocity0, desiredVelocity1, blendingFactor);
         velocityStepSizeLimiter.updateOutput(currentJointVelocity, desiredVelocity);
         outputDataToPack.setDesiredVelocity(velocityStepSizeLimiter.getDoubleValue());
      }

      if (hasDesiredAcceleration(outputData0) || hasDesiredAcceleration(outputData1))
      {
         double desiredAcceleration0 = hasDesiredAcceleration(outputData0) ? outputData0.getDesiredAcceleration() : 0.0;
         double desiredAcceleration1 = hasDesiredAcceleration(outputData1) ? outputData1.getDesiredAcceleration() : 0.0;
         double desiredAcceleration = TupleTools.interpolate(desiredAcceleration0, desiredAcceleration1, blendingFactor);
         outputDataToPack.setDesiredAcceleration(desiredAcceleration);
      }

      if (hasDesiredTorque(outputData0) || hasDesiredTorque(outputData1))
      {
         double desiredTorque0 = hasDesiredTorque(outputData0) ? outputData0.getDesiredTorque() : 0.0;
         double desiredTorque1 = hasDesiredTorque(outputData1) ? outputData1.getDesiredTorque() : 0.0;
         double desiredTorque = TupleTools.interpolate(desiredTorque0, desiredTorque1, blendingFactor);
         outputDataToPack.setDesiredTorque(desiredTorque);
      }

      if (hasStiffness(outputData0) || hasStiffness(outputData1))
      {
         double stiffness0 = hasStiffness(outputData0) ? outputData0.getStiffness() : 0.0;
         double stiffness1 = hasStiffness(outputData1) ? outputData1.getStiffness() : 0.0;
         double stiffness = TupleTools.interpolate(stiffness0, stiffness1, blendingFactor);
         outputDataToPack.setStiffness(stiffness);
      }

      if (hasDamping(outputData0) || hasDamping(outputData1))
      {
         double damping0 = hasDamping(outputData0) ? outputData0.getDamping() : 0.0;
         double damping1 = hasDamping(outputData1) ? outputData1.getDamping() : 0.0;
         double damping = TupleTools.interpolate(damping0, damping1, blendingFactor);
         outputDataToPack.setDamping(damping);
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

   private boolean hasStiffness(JointDesiredOutputReadOnly outputData)
   {
      return outputData != null && outputData.hasStiffness();
   }

   private boolean hasDamping(JointDesiredOutputReadOnly outputData)
   {
      return outputData != null && outputData.hasDamping();
   }

   private boolean pollResetIntegratorsRequest(JointDesiredOutputReadOnly outputData)
   {
      return outputData != null && outputData.pollResetIntegratorsRequest();
   }
}
