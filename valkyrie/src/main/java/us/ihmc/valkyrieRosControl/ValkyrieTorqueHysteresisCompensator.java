package us.ihmc.valkyrieRosControl;

import java.util.ArrayList;
import java.util.List;

import org.apache.commons.lang3.StringUtils;

import us.ihmc.commonWalkingControlModules.controllerCore.command.lowLevel.YoJointDesiredOutput;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputReadOnly;
import us.ihmc.valkyrieRosControl.dataHolders.YoEffortJointHandleHolder;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class ValkyrieTorqueHysteresisCompensator
{
   private final YoVariableRegistry registry = new YoVariableRegistry("TorqueHysteresisCompensator");

   private final List<YoEffortJointHandleHolder> processedJointHandles = new ArrayList<>();
   private final List<TorqueHysteresisCompensatorYoVariable> hysteresisCompensators = new ArrayList<>();

   private final YoDouble torqueHysteresisAmplitude = new YoDouble("torqueHysteresisAmplitude", registry);
   private final YoDouble jointAccelerationMin = new YoDouble("hysteresisJointAccelerationMin", registry);
   private final YoDouble jointVelocityMax = new YoDouble("hysteresisJointVelocityMax", registry);

   private final YoDouble rampUpTime = new YoDouble("torqueHysteresisRampUpTime", registry);
   private final YoDouble rampDownTime = new YoDouble("torqueHysteresisRampDownTime", registry);

   /**
    * Need to be extracted to a config file
    */
   private final String[] jointShortNamesToProcess = new String[]{"Hip", "Knee", "Torso", "Shoulder", "Elbow"};

   public ValkyrieTorqueHysteresisCompensator(List<YoEffortJointHandleHolder> yoEffortJointHandleHolders, YoDouble yoTime, YoVariableRegistry parentRegistry)
   {
      parentRegistry.addChild(registry);

      torqueHysteresisAmplitude.set(1.5);
      jointAccelerationMin.set(1.0);
      jointVelocityMax.set(0.1);
      rampUpTime.set(1.0);
      rampDownTime.set(0.1);

      for (YoEffortJointHandleHolder jointHandle : yoEffortJointHandleHolders)
      {
         OneDoFJoint joint = jointHandle.getOneDoFJoint();
         String jointName = joint.getName();
         if (!shouldProcessJoint(jointName))
            continue;

         JointDesiredOutputReadOnly lowLevelJointData = jointHandle.getDesiredJointData();
         processedJointHandles.add(jointHandle);
         TorqueHysteresisCompensatorYoVariable hysteresisCompensator = new TorqueHysteresisCompensatorYoVariable("tau_offHyst_", joint, lowLevelJointData, torqueHysteresisAmplitude, jointAccelerationMin, jointVelocityMax, rampUpTime, rampDownTime, yoTime, registry);
         hysteresisCompensator.enable();
         hysteresisCompensators.add(hysteresisCompensator);
      }
   }

   public void compute()
   {
      for (int i = 0; i < processedJointHandles.size(); i++)
      {
         TorqueHysteresisCompensatorYoVariable hysteresisCompensator = hysteresisCompensators.get(i);
         YoEffortJointHandleHolder jointHandle = processedJointHandles.get(i);
         
         hysteresisCompensator.update();
         
         YoJointDesiredOutput desiredJointData = jointHandle.getDesiredJointData();
         double desiredTorque = desiredJointData.getDesiredTorque();
         desiredTorque += hysteresisCompensator.getDoubleValue();
         desiredJointData.setDesiredTorque(desiredTorque);
      }
   }

   private boolean shouldProcessJoint(String jointName)
   {
      boolean processJoint = false;
      for (String shortName : jointShortNamesToProcess)
      {
         if (StringUtils.containsIgnoreCase(jointName, shortName))
         {
            processJoint = true;
            break;
         }
      }
      return processJoint;
   }
}
