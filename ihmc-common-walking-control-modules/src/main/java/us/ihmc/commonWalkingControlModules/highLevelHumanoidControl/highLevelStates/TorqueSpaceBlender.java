package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates;

import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.log.LogTools;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputBasics;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputReadOnly;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class TorqueSpaceBlender implements CommandBlender
{
   private final OneDoFJointBasics joint;
   private final YoDouble equivalentError;
   private final YoRegistry registry;

   public TorqueSpaceBlender(OneDoFJointBasics joint, YoRegistry parentRegistry)
   {
      this.joint = joint;
      String prefix = joint.getName();
      this.registry = new YoRegistry(prefix + "TorqueSpaceBlender");
      equivalentError = new YoDouble(joint.getName() + "EquivalentError", registry);

      if (parentRegistry != null)
         parentRegistry.addChild(registry);
   }

   public void computeAndUpdateJointControl(JointDesiredOutputBasics outputToPack,
                                            JointDesiredOutputReadOnly from,
                                            JointDesiredOutputReadOnly to,
                                            double alpha)
   {
      outputToPack.clear();
      outputToPack.setControlMode(from.getControlMode());
      outputToPack.setLoadMode(from.getLoadMode());

      boolean validFrom = from.hasDesiredPosition() && from.hasStiffness() && from.hasDamping();
      boolean validTo = to.hasDesiredPosition() && to.hasStiffness() && to.hasDamping();

      if (validTo && validFrom)
      {
         double pseudoStandPrepTorque =
               from.getStiffness() * (from.getDesiredPosition() - joint.getQ()) - from.getDamping() * joint.getQd();
         double pseudoWalkingTorque =
               to.getStiffness() * (to.getDesiredPosition() - joint.getQ()) - to.getDamping() * joint.getQd();

         double effectiveTorque = EuclidCoreTools.interpolate(pseudoStandPrepTorque, pseudoWalkingTorque, alpha);
         double effectiveStiffness = EuclidCoreTools.interpolate(from.getStiffness(), to.getStiffness(), alpha);
         double effectiveDamping = EuclidCoreTools.interpolate(from.getDamping(), to.getDamping(), alpha);

         double viscousDampingTorque = -effectiveDamping * joint.getQd();
         double neededStiffnessTorque = effectiveTorque - viscousDampingTorque;

         if (effectiveStiffness == 0.0)
         {
            LogTools.warn("Effective stiffness is zero for joint {}. Setting desired position = current position.",
                          joint.getName());
            outputToPack.setDesiredPosition(joint.getQ());
            outputToPack.setDesiredVelocity(0.0);
            outputToPack.setStiffness(effectiveStiffness);
            outputToPack.setDamping(effectiveDamping);
            return;
         }

         equivalentError.set(neededStiffnessTorque / effectiveStiffness);
         // tau = stiffnessFeedback + dampingFeedback -> stiffnessFeedback = tau - dampingFeedback -> positionError = (stiffnessfeedback + dampingFeedback) / effectiveStiffness
         double positionSetpoint = equivalentError.getDoubleValue() + joint.getQ();

         outputToPack.setStiffness(effectiveStiffness);
         outputToPack.setDamping(effectiveDamping);
         outputToPack.setDesiredPosition(positionSetpoint);
         outputToPack.setDesiredVelocity(0.0);
      }
      else if (validTo)
      {
         LogTools.info("From output wasn't valid, setting to to.");
         outputToPack.set(to);
      }
      else if (validFrom)
      {
         LogTools.info("To wasn't valid, setting to from.");
         outputToPack.set(from);
      }
   }
}