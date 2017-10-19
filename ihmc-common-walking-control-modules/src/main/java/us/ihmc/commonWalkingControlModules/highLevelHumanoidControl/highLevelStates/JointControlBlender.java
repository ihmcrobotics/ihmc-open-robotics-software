package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates;

import us.ihmc.euclid.tools.TupleTools;
import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.math.filters.DeltaLimitedYoVariable;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
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

      double desiredPosition0 = outputData0.hasDesiredPosition() ? outputData0.getDesiredPosition() : oneDoFJoint.getQ();
      double desiredPosition1 = outputData1.hasDesiredPosition() ? outputData1.getDesiredPosition() : oneDoFJoint.getQ();

      double desiredVelocity0 = outputData0.hasDesiredVelocity() ? outputData0.getDesiredVelocity() : oneDoFJoint.getQd();
      double desiredVelocity1 = outputData1.hasDesiredVelocity() ? outputData1.getDesiredVelocity() : oneDoFJoint.getQd();

      double desiredAcceleration0 = outputData0.hasDesiredAcceleration() ? outputData0.getDesiredAcceleration() : 0.0;
      double desiredAcceleration1 = outputData1.hasDesiredAcceleration() ? outputData1.getDesiredAcceleration() : 0.0;

      double desiredTorque0 = outputData0.hasDesiredTorque() ? outputData0.getDesiredTorque() : 0.0;
      double desiredTorque1 = outputData1.hasDesiredTorque() ? outputData1.getDesiredTorque() : 0.0;

      double desiredPosition = TupleTools.interpolate(desiredPosition0, desiredPosition1, blendingFactor);
      double desiredVelocity = TupleTools.interpolate(desiredVelocity0, desiredVelocity1, blendingFactor);
      double desiredAcceleration = TupleTools.interpolate(desiredAcceleration0, desiredAcceleration1, blendingFactor);
      double desiredTorque = TupleTools.interpolate(desiredTorque0, desiredTorque1, blendingFactor);

      double currentJointAngle = oneDoFJoint.getQ();
      double currentJointVelocity = oneDoFJoint.getQd();
      positionStepSizeLimiter.updateOutput(currentJointAngle, desiredPosition);
      velocityStepSizeLimiter.updateOutput(currentJointVelocity, desiredVelocity);

      outputDataToPack.setDesiredPosition(positionStepSizeLimiter.getDoubleValue());
      outputDataToPack.setDesiredVelocity(velocityStepSizeLimiter.getDoubleValue());
      outputDataToPack.setDesiredAcceleration(desiredAcceleration);
      outputDataToPack.setDesiredTorque(desiredTorque);
   }
}
