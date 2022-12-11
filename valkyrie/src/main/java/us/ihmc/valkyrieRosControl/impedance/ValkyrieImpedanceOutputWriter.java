package us.ihmc.valkyrieRosControl.impedance;

import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.JointspacePositionControllerState;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.rosControl.EffortJointHandle;
import us.ihmc.rosControl.wholeRobot.JointImpedanceHandle;
import us.ihmc.valkyrieRosControl.ValkyrieTorqueOffsetPrinter;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

import java.util.HashMap;
import java.util.List;
import java.util.Map;

import static us.ihmc.valkyrieRosControl.impedance.ValkyrieWholeBodyImpedanceController.allJoints;
import static us.ihmc.valkyrieRosControl.impedance.ValkyrieWholeBodyImpedanceController.impedanceJoints;
import static us.ihmc.valkyrieRosControl.impedance.ValkyrieWholeBodyImpedanceController.torqueJoints;

public class ValkyrieImpedanceOutputWriter
{
   private static final Map<String, Double> torqueOffsets = ValkyrieTorqueOffsetPrinter.loadTorqueOffsetsFromFile();

   static
   {
      if (torqueOffsets == null)
      {
         throw new RuntimeException("Torque offsets file could not load");
      }
   }

   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());
   private final FullHumanoidRobotModel fullRobotModel;

   /* For joints with impedance API */
   private final YoDouble masterGain = new YoDouble("masterGain", registry);
   private final YoDouble desiredJointStiffness = new YoDouble("desiredJointStiffness", registry);
   private final YoDouble desiredJointDamping = new YoDouble("desiredJointDamping", registry);

   /* For joints without impedance API */
   private final YoDouble highLevelMasterGain = new YoDouble("highLevelMasterGain", registry);
   private final YoDouble highLevelJointStiffness = new YoDouble("highLevelJointStiffness", registry);
   private final YoDouble highLevelJointDamping = new YoDouble("highLevelJointDamping", registry);

   private final Map<String, JointspacePositionControllerState.OneDoFJointManager> nameToJointManagerMap;
   private final Map<String, EffortJointHandle> nameToEffortHandleMap;
   private final Map<String, JointImpedanceHandle> nameToImpedanceHandleMap;

   public ValkyrieImpedanceOutputWriter(FullHumanoidRobotModel fullRobotModel,
                                        Map<String, JointspacePositionControllerState.OneDoFJointManager> nameToJointManagerMap,
                                        Map<String, EffortJointHandle> nameToEffortHandleMap,
                                        Map<String, JointImpedanceHandle> nameToImpedanceHandleMap)
   {
      this.fullRobotModel = fullRobotModel;

      this.nameToJointManagerMap = nameToJointManagerMap;
      this.nameToEffortHandleMap = nameToEffortHandleMap;
      this.nameToImpedanceHandleMap = nameToImpedanceHandleMap;

      masterGain.set(0.1);
      desiredJointStiffness.set(200.0);
      desiredJointDamping.set(35.0);

      highLevelMasterGain.set(0.6);
      highLevelJointStiffness.set(55.0);
      highLevelJointDamping.set(6.0);
   }

   public void write()
   {
      for (int i = 0; i < allJoints.size(); i++)
      {
         String jointName = allJoints.get(i);
         JointspacePositionControllerState.OneDoFJointManager jointManager = nameToJointManagerMap.get(jointName);
         EffortJointHandle effortJointHandle = nameToEffortHandleMap.get(jointName);
         double torqueOffset = torqueOffsets.get(jointName);

         if (impedanceJoints.contains(jointName))
         {
            JointImpedanceHandle impedanceHandle = nameToImpedanceHandleMap.get(jointName);

            impedanceHandle.setStiffness(masterGain.getDoubleValue() * desiredJointStiffness.getDoubleValue());
            impedanceHandle.setDamping(masterGain.getDoubleValue() * desiredJointDamping.getDoubleValue());
            impedanceHandle.setPosition(jointManager.getJointDesiredPosition());
            impedanceHandle.setVelocity(jointManager.getJointDesiredVelocity());
            effortJointHandle.setDesiredEffort(-torqueOffset);
         }
         else if (torqueJoints.contains(jointName))
         {
            OneDoFJointBasics joint = fullRobotModel.getOneDoFJointByName(jointName);
            double q = joint.getQ();
            double qd = joint.getQd();
            double qDes = jointManager.getJointDesiredPosition();
            double qdDes = jointManager.getJointDesiredVelocity();

            double kp = highLevelMasterGain.getDoubleValue() * highLevelJointStiffness.getDoubleValue();
            double kd = highLevelMasterGain.getDoubleValue() * highLevelJointDamping.getDoubleValue();
            double effort = -torqueOffset + kp * (qDes - q) + kd * (qdDes - qd);
            effortJointHandle.setDesiredEffort(effort);
         }
         else
         {
            throw new RuntimeException("Joint " + jointName + " is not registered with either the impedance or torque lists");
         }
      }
   }
}
