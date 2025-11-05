package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates;

import us.ihmc.commons.MathTools;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.log.LogTools;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputBasics;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputReadOnly;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class RLJointControlBlender implements CommandBlender
{
   private final YoDouble equivalentError;
   private final OneDoFJointBasics oneDoFJoint;

   public RLJointControlBlender(String nameSuffix, OneDoFJointBasics oneDoFJoint, YoRegistry parentRegistry)
   {
      this.oneDoFJoint = oneDoFJoint;
      String namePrefix = oneDoFJoint.getName();

      YoRegistry registry = new YoRegistry(namePrefix + nameSuffix + "JointControlBlender");
      equivalentError = new YoDouble(oneDoFJoint.getName() + "EquivalentError", registry);

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
      LogTools.warn(blendingFactor);
      blendingFactor = MathTools.clamp(blendingFactor, 0.0, 1.0);

      outputDataToPack.clear();

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

      outputDataToPack.setControlMode(outputData1.getControlMode());
      outputDataToPack.setLoadMode(outputData1.getLoadMode());

      boolean validFrom = outputData0.hasDesiredPosition() && outputData0.hasStiffness() && outputData0.hasDamping();
      boolean validTo = outputData1.hasDesiredPosition() && outputData1.hasStiffness() && outputData1.hasDamping();

      if (validTo && validFrom)
      {
         double pseudoStandPrepTorque = outputData0.getStiffness() * (outputData0.getDesiredPosition() - oneDoFJoint.getQ()) - outputData0.getDamping() * oneDoFJoint.getQd();
         double pseudoWalkingTorque = outputData1.getStiffness() * (outputData1.getDesiredPosition() - oneDoFJoint.getQ()) - outputData1.getDamping() * oneDoFJoint.getQd();

         double effectiveTorque = EuclidCoreTools.interpolate(pseudoStandPrepTorque, pseudoWalkingTorque, blendingFactor);
         double effectiveStiffness = EuclidCoreTools.interpolate(outputData0.getStiffness(), outputData1.getStiffness(), blendingFactor);
         double effectiveDamping = EuclidCoreTools.interpolate(outputData0.getDamping(), outputData1.getDamping(), blendingFactor);

         double viscousDampingTorque = -effectiveDamping * oneDoFJoint.getQd();
         double neededStiffnessTorque = effectiveTorque - viscousDampingTorque;
         equivalentError.set(neededStiffnessTorque / effectiveStiffness);
         // tau = stiffnessFeedback + dampingFeedback -> stiffnessFeedback = tau - dampingFeedback -> positionError = (stiffnessfeedback + dampingFeedback) / effectiveStiffness
         double positionSetpoint = equivalentError.getDoubleValue() + oneDoFJoint.getQ();

         outputDataToPack.setStiffness(effectiveStiffness);
         outputDataToPack.setDamping(effectiveDamping);
         outputDataToPack.setDesiredPosition(positionSetpoint);
         outputDataToPack.setDesiredVelocity(0.0);
      }
      else if (validTo)
      {
         LogTools.info("From output wasn't valid, setting to to.");
         outputDataToPack.set(outputData1);
      }
      else if (validFrom)
      {
         LogTools.info("To wasn't valid, setting to from.");
         outputDataToPack.set(outputData0);
      }
   }
}
