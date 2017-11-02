package us.ihmc.valkyrieRosControl;

import java.util.ArrayList;
import java.util.List;

import org.apache.commons.lang3.StringUtils;

import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutput;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputReadOnly;
import us.ihmc.valkyrieRosControl.dataHolders.YoEffortJointHandleHolder;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class ValkyrieAccelerationIntegration
{
   private final YoVariableRegistry registry = new YoVariableRegistry("AccelerationIntegration");

   private final List<YoEffortJointHandleHolder> processedJointHandles = new ArrayList<>();

   private final List<YoDouble> kPositionList = new ArrayList<>();
   private final List<YoDouble> kVelocityList = new ArrayList<>();

   private final List<YoDouble> tauFromPositionList = new ArrayList<>();
   private final List<YoDouble> tauFromVelocityList = new ArrayList<>();

   /**
    * Need to be extracted to a config file
    */
   private final String[] legs = new String[] {"Hip", "Knee", "Ankle"};
   private final String[] upperarms = new String[] {"Shoulder", "Elbow"};
   private final String[] forearms = new String[] {"Forearm", "Wrist"};

   public ValkyrieAccelerationIntegration(List<YoEffortJointHandleHolder> yoEffortJointHandleHolders, double updateDT, YoVariableRegistry parentRegistry)
   {
      parentRegistry.addChild(registry);

      for (YoEffortJointHandleHolder jointHandle : yoEffortJointHandleHolders)
      {
         OneDoFJoint joint = jointHandle.getOneDoFJoint();
         String jointName = joint.getName();

         processedJointHandles.add(jointHandle);

         YoDouble kPosition = new YoDouble("kPosition_" + jointName, registry);
         YoDouble kVelocity = new YoDouble("kVelocity_" + jointName, registry);
         YoDouble tauFromPosition = new YoDouble("tau_pos_" + jointName, registry);
         YoDouble tauFromVelocity = new YoDouble("tau_vel_" + jointName, registry);

         kPositionList.add(kPosition);
         kVelocityList.add(kVelocity);
         tauFromPositionList.add(tauFromPosition);
         tauFromVelocityList.add(tauFromVelocity);
      }
      setGains();
   }

   /**
    * Totally needs to be extracted.
    */
   private void setGains()
   {
      for (int i = 0; i < processedJointHandles.size(); i++)
      {
         YoEffortJointHandleHolder jointHandle = processedJointHandles.get(i);
         OneDoFJoint joint = jointHandle.getOneDoFJoint();
         String jointName = joint.getName();

         if (arrayContainsIgnoreCase(legs, jointName))
         {
            kVelocityList.get(i).set(4.75);
         }
         else if (arrayContainsIgnoreCase(upperarms, jointName))
         {
            kVelocityList.get(i).set(6.0);
         }
         else if (jointName.contains("ForearmYaw"))
         {
            kPositionList.get(i).set(7.0);
         }
         else if (jointName.contains("Wrist"))
         {
            kPositionList.get(i).set(20.0);
            kVelocityList.get(i).set(0.5);
         }
      }
   }

   public void compute()
   {
      for (int i = 0; i < processedJointHandles.size(); i++)
      {
         YoEffortJointHandleHolder jointHandle = processedJointHandles.get(i);
         JointDesiredOutput desiredJointData = jointHandle.getDesiredJointData();
         computeAndApplyDesiredTauOffset(i, desiredJointData);
      }
   }

   private void computeAndApplyDesiredTauOffset(int index, JointDesiredOutputReadOnly desiredJointData)
   {
      YoEffortJointHandleHolder jointHandle = processedJointHandles.get(index);
      OneDoFJoint joint = jointHandle.getOneDoFJoint();

      double tau_pos;

      if (desiredJointData.hasDesiredPosition())
      {
         double q = joint.getQ();
         double q_d = desiredJointData.getDesiredPosition();
         double kp = kPositionList.get(index).getDoubleValue();
         tau_pos = kp * (q_d - q);
      }
      else
      {
         tau_pos = 0.0;
      }
      tauFromPositionList.get(index).set(tau_pos);

      double tau_vel;
      if (desiredJointData.hasDesiredVelocity())
      {
         double qd = joint.getQd();
         double qd_d = desiredJointData.getDesiredVelocity();
         double kd = kVelocityList.get(index).getDoubleValue();
         tau_vel = kd * (qd_d - qd);
      }
      else
      {
         tau_vel = 0.0;
      }
      tauFromVelocityList.get(index).set(tau_vel);

      jointHandle.addOffetControllerTauDesired(tau_pos + tau_vel);
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
