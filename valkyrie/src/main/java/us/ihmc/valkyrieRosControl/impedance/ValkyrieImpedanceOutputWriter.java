package us.ihmc.valkyrieRosControl.impedance;

import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.JointspacePositionControllerState;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.rosControl.EffortJointHandle;
import us.ihmc.rosControl.wholeRobot.JointImpedanceHandle;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputBasics;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputListBasics;
import us.ihmc.valkyrieRosControl.ValkyrieTorqueOffsetPrinter;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

import java.util.Map;

import static us.ihmc.valkyrieRosControl.impedance.ValkyrieJointList.*;

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

   private final FullHumanoidRobotModel fullRobotModel;
   private final JointDesiredOutputListBasics jointDesiredOutputList;
   private final Map<String, EffortJointHandle> nameToEffortHandleMap;
   private final Map<String, JointImpedanceHandle> nameToImpedanceHandleMap;

   public ValkyrieImpedanceOutputWriter(FullHumanoidRobotModel fullRobotModel,
                                        JointDesiredOutputListBasics jointDesiredOutputList,
                                        Map<String, EffortJointHandle> nameToEffortHandleMap,
                                        Map<String, JointImpedanceHandle> nameToImpedanceHandleMap)
   {
      this.fullRobotModel = fullRobotModel;

      this.jointDesiredOutputList = jointDesiredOutputList;
      this.nameToEffortHandleMap = nameToEffortHandleMap;
      this.nameToImpedanceHandleMap = nameToImpedanceHandleMap;
   }

   public void write()
   {
      for (int i = 0; i < allJoints.size(); i++)
      {
         String jointName = allJoints.get(i);
         OneDoFJointBasics joint = fullRobotModel.getOneDoFJointByName(jointName);
         JointDesiredOutputBasics jointDesiredOutput = jointDesiredOutputList.getJointDesiredOutput(joint);
         EffortJointHandle effortJointHandle = nameToEffortHandleMap.get(jointName);
         double torqueOffset = torqueOffsets.get(jointName);

         double masterGain = jointDesiredOutput.getMasterGain();
         double stiffness = jointDesiredOutput.getStiffness();
         double damping = jointDesiredOutput.getDamping();

         if (impedanceJoints.contains(jointName))
         {
            JointImpedanceHandle impedanceHandle = nameToImpedanceHandleMap.get(jointName);

            impedanceHandle.setStiffness(masterGain * stiffness);
            impedanceHandle.setDamping(masterGain * damping);
            impedanceHandle.setPosition(jointDesiredOutput.getDesiredPosition());
            impedanceHandle.setVelocity(jointDesiredOutput.getDesiredVelocity());
            effortJointHandle.setDesiredEffort(-torqueOffset);
         }
         else if (torqueJoints.contains(jointName))
         {
            double q = joint.getQ();
            double qd = joint.getQd();
            double qDes = jointDesiredOutput.getDesiredPosition();
            double qdDes = jointDesiredOutput.getDesiredVelocity();

            double kp = masterGain * stiffness;
            double kd = masterGain * damping;
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
