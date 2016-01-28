package us.ihmc.valkyrieRosControl;

import java.util.ArrayList;
import java.util.List;

import org.apache.commons.lang3.StringUtils;

import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.valkyrieRosControl.dataHolders.YoJointHandleHolder;

public class ValkyrieTorqueHysteresisCompensator
{
   private final YoVariableRegistry registry = new YoVariableRegistry("TorqueHysteresisCompensator");

   private final List<YoJointHandleHolder> processedJointHandles = new ArrayList<>();
   private final List<TorqueHysteresisCompensatorYoVariable> hysteresisCompensators = new ArrayList<>();

   private final DoubleYoVariable torqueHysteresisAmplitude = new DoubleYoVariable("torqueHysteresisAmplitude", registry);
   private final DoubleYoVariable jointAccelerationMin = new DoubleYoVariable("hysteresisJointAccelerationMin", registry);
   private final DoubleYoVariable jointVelocityMax = new DoubleYoVariable("hysteresisJointVelocityMax", registry);

   private final DoubleYoVariable rampUpTime = new DoubleYoVariable("torqueHysteresisRampUpTime", registry);
   private final DoubleYoVariable rampDownTime = new DoubleYoVariable("torqueHysteresisRampDownTime", registry);

   /**
    * Need to be extracted to a config file
    */
   private final String[] jointShortNamesToProcess = new String[]{"Hip", "Knee", "Torso", "Shoulder", "Elbow"};

   public ValkyrieTorqueHysteresisCompensator(List<YoJointHandleHolder> yoJointHandleHolders, DoubleYoVariable yoTime, YoVariableRegistry parentRegistry)
   {
      parentRegistry.addChild(registry);

      torqueHysteresisAmplitude.set(1.5);
      jointAccelerationMin.set(1.0);
      jointVelocityMax.set(0.1);
      rampUpTime.set(1.0);
      rampDownTime.set(0.1);

      for (YoJointHandleHolder jointHandle : yoJointHandleHolders)
      {
         OneDoFJoint joint = jointHandle.getOneDoFJoint();
         String jointName = joint.getName();
         if (!shouldProcessJoint(jointName))
            continue;

         processedJointHandles.add(jointHandle);
         TorqueHysteresisCompensatorYoVariable hysteresisCompensator = new TorqueHysteresisCompensatorYoVariable("tau_offHyst_", joint, torqueHysteresisAmplitude, jointAccelerationMin, jointVelocityMax, rampUpTime, rampDownTime, yoTime, registry);
         hysteresisCompensator.enable();
         hysteresisCompensators.add(hysteresisCompensator);
      }
   }

   public void compute()
   {
      for (int i = 0; i < processedJointHandles.size(); i++)
      {
         TorqueHysteresisCompensatorYoVariable hysteresisCompensator = hysteresisCompensators.get(i);
         YoJointHandleHolder jointHandle = processedJointHandles.get(i);
         
         jointHandle.getOneDoFJoint().setQddDesired(jointHandle.getControllerQddDesired());
         hysteresisCompensator.update();
         
         jointHandle.addOffetControllerTauDesired(hysteresisCompensator.getDoubleValue());
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
