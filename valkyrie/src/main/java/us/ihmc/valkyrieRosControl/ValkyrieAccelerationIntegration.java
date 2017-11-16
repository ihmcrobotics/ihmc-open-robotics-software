package us.ihmc.valkyrieRosControl;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import us.ihmc.robotics.partNames.ArmJointName;
import us.ihmc.robotics.partNames.LegJointName;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutput;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputReadOnly;
import us.ihmc.valkyrie.parameters.ValkyrieJointMap;
import us.ihmc.valkyrieRosControl.dataHolders.YoEffortJointHandleHolder;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;

public class ValkyrieAccelerationIntegration
{
   private final YoVariableRegistry registry = new YoVariableRegistry("AccelerationIntegration");

   private final List<YoEffortJointHandleHolder> processedJointHandles = new ArrayList<>();

   private final YoBoolean useRawPosition = new YoBoolean("useRawPositionForAccelerationIntegration", registry);
   private final YoBoolean useRawVelocity = new YoBoolean("useRawVelocityForAccelerationIntegration", registry);

   private final List<YoDouble> kPositionList = new ArrayList<>();
   private final List<YoDouble> kVelocityList = new ArrayList<>();

   private final List<YoDouble> tauFromPositionList = new ArrayList<>();
   private final List<YoDouble> tauFromVelocityList = new ArrayList<>();

   public ValkyrieAccelerationIntegration(ValkyrieJointMap jointMap, List<YoEffortJointHandleHolder> yoEffortJointHandleHolders, double updateDT,
                                          YoVariableRegistry parentRegistry)
   {
      // The damping seems to be working better using the raw velocity measurement
      useRawVelocity.set(true);
      parentRegistry.addChild(registry);

      Map<String, YoDouble> kPositionMap = new HashMap<>();
      Map<String, YoDouble> kVelocityMap = new HashMap<>();

      for (LegJointName legJointName : jointMap.getLegJointNames())
      {
         YoDouble kPosition = new YoDouble("kPosition_" + legJointName.getCamelCaseName(), registry);
         YoDouble kVelocity = new YoDouble("kVelocity_" + legJointName.getCamelCaseName(), registry);
         
         for (RobotSide robotSide : RobotSide.values)
         {
            kPositionMap.put(jointMap.getLegJointName(robotSide, legJointName), kPosition);
            kVelocityMap.put(jointMap.getLegJointName(robotSide, legJointName), kVelocity);
         }
      }

      for (ArmJointName armJointName : jointMap.getArmJointNames())
      {
         YoDouble kPosition = new YoDouble("kPosition_" + armJointName.getCamelCaseNameForStartOfExpression(), registry);
         YoDouble kVelocity = new YoDouble("kVelocity_" + armJointName.getCamelCaseNameForStartOfExpression(), registry);
         
         for (RobotSide robotSide : RobotSide.values)
         {
            kPositionMap.put(jointMap.getArmJointName(robotSide, armJointName), kPosition);
            kVelocityMap.put(jointMap.getArmJointName(robotSide, armJointName), kVelocity);
         }
      }

      for (YoEffortJointHandleHolder jointHandle : yoEffortJointHandleHolders)
      {
         OneDoFJoint joint = jointHandle.getOneDoFJoint();
         String jointName = joint.getName();

         processedJointHandles.add(jointHandle);

         YoDouble kPosition = kPositionMap.get(jointName);
         if (kPosition == null)
            kPosition = new YoDouble("kPosition_" + jointName, registry);
         YoDouble kVelocity = kVelocityMap.get(jointName);
         if (kVelocity == null)
            kVelocity = new YoDouble("kVelocity_" + jointName, registry);
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

         if (jointName.contains("Hip"))
         {
            kVelocityList.get(i).set(30.0);
            kPositionList.get(i).set(3.0);
         }
         else if (jointName.contains("Knee"))
         {
            kVelocityList.get(i).set(30.0);
            kPositionList.get(i).set(4.0);
         }
         else if (jointName.contains("Ankle"))
         {
            kVelocityList.get(i).set(60.0);
            kPositionList.get(i).set(6.0);
         }
         else if (jointName.contains("torso"))
         {
            kVelocityList.get(i).set(50.0);
            kPositionList.get(i).set(1.0);
         }
         else if (jointName.contains("Shoulder"))
         {
            kVelocityList.get(i).set(30.0);
            kPositionList.get(i).set(2.0);
         }
         else if (jointName.contains("Elbow"))
         {
            kVelocityList.get(i).set(30.0);
            kPositionList.get(i).set(2.0);
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
         double q = useRawPosition.getBooleanValue() ? jointHandle.getQ() : joint.getQ();
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
         double qd = useRawVelocity.getBooleanValue() ? jointHandle.getQd() : joint.getQd();
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
}
