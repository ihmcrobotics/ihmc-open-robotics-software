package us.ihmc.valkyrieRosControl;

import java.util.ArrayList;
import java.util.List;

import org.apache.commons.lang3.StringUtils;

import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.valkyrieRosControl.dataHolders.YoEffortJointHandleHolder;

public class ValkyrieAccelerationIntegration
{
   private final YoVariableRegistry registry = new YoVariableRegistry("AccelerationIntegration");

   private final List<YoEffortJointHandleHolder> processedJointHandles = new ArrayList<>();

   private final YoDouble alphaAccelerationIntegration = new YoDouble("alphaAccelerationIntegration", registry);

   private final List<YoDouble> kVelocityList = new ArrayList<>();

   private final List<YoDouble> desiredVelocityList = new ArrayList<>();

   private final List<YoDouble> tauFromVelocityList = new ArrayList<>();

   private final List<YoBoolean> enabledList = new ArrayList<>();

   private final double updateDT;

   /**
    * Need to be extracted to a config file
    */
   private final String[] jointShortNamesToProcess = new String[]{"Hip", "Knee", "Ankle", "Shoulder", "Elbow", "Torso"};
   private final String[] legs = new String[]{"Hip", "Knee", "Ankle"};
   private final String[] arms = new String[]{"Shoulder", "Elbow"};

   public ValkyrieAccelerationIntegration(List<YoEffortJointHandleHolder> yoEffortJointHandleHolders, double updateDT, YoVariableRegistry parentRegistry)
   {
      this.updateDT = updateDT;
      parentRegistry.addChild(registry);

      for (YoEffortJointHandleHolder jointHandle : yoEffortJointHandleHolders)
      {
         OneDoFJoint joint = jointHandle.getOneDoFJoint();
         String jointName = joint.getName();
         if (!arrayContainsIgnoreCase(jointShortNamesToProcess, jointName))
            continue;

         processedJointHandles.add(jointHandle);

         YoDouble kVelocity = new YoDouble("kVelocity_" + jointName, registry);
         YoDouble desiredVelocity = new YoDouble("qd_d_AccInt_" + jointName, registry);
         YoDouble tauFromVelocity = new YoDouble("tau_vel_" + jointName, registry);
         YoBoolean enabled = new YoBoolean(jointName + "AccelIntEnabled", registry);

         kVelocityList.add(kVelocity);
         desiredVelocityList.add(desiredVelocity);
         tauFromVelocityList.add(tauFromVelocity);
         enabledList.add(enabled);
      }
      setGains();
   }

   /**
    * Totally needs to be extracted.
    */
   private void setGains()
   {
      alphaAccelerationIntegration.set(0.9);

      for (int i = 0; i < processedJointHandles.size(); i++)
      {
         YoEffortJointHandleHolder jointHandle = processedJointHandles.get(i);
         OneDoFJoint joint = jointHandle.getOneDoFJoint();
         String jointName = joint.getName();

         if (arrayContainsIgnoreCase(legs, jointName))
         {
            kVelocityList.get(i).set(4.75);
         }
         else if (arrayContainsIgnoreCase(arms, jointName))
         {
            kVelocityList.get(i).set(6.0);
         }
      }
   }

   public void compute()
   {
      for (int i = 0; i < processedJointHandles.size(); i++)
      {
         YoEffortJointHandleHolder jointHandle = processedJointHandles.get(i);
         OneDoFJoint joint = jointHandle.getOneDoFJoint();

         boolean enabled = joint.getIntegrateDesiredAccelerations();
         enabledList.get(i).set(enabled);
         
         if (enabled && !joint.getResetDesiredAccelerationIntegrator())
            computeAndApplyDesiredTauOffset(i);
         else
            reset(i);
      }
   }

   private void computeAndApplyDesiredTauOffset(int index)
   {
      YoEffortJointHandleHolder jointHandle = processedJointHandles.get(index);
      OneDoFJoint joint = jointHandle.getOneDoFJoint();
      double alpha = alphaAccelerationIntegration.getDoubleValue();
      double qd = joint.getQd();
      double qd_d_previous = desiredVelocityList.get(index).getDoubleValue();
      double qdd_d = jointHandle.getControllerQddDesired();

      double qd_d_new = alpha * (qd_d_previous + qdd_d * updateDT) + (1.0 - alpha) * qd;
      double tau_vel = kVelocityList.get(index).getDoubleValue() * (qd_d_new - qd);

      jointHandle.addOffetControllerTauDesired(tau_vel);

      desiredVelocityList.get(index).set(qd_d_new);
      tauFromVelocityList.get(index).set(tau_vel);
   }

   private void reset(int index)
   {
      desiredVelocityList.get(index).set(0.0);
   }

   private boolean arrayContainsIgnoreCase(String[] arrayToCheck, String searchString)
   {
      for (String shortName : arrayToCheck)
      {
         if (StringUtils.containsIgnoreCase(searchString, shortName))
            return true;
      }
      return false;
   }
}
