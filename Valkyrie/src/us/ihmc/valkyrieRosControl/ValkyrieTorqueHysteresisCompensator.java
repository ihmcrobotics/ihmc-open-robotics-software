package us.ihmc.valkyrieRosControl;

import java.util.ArrayList;
import java.util.List;

import org.apache.commons.lang3.StringUtils;

import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.ScrewTools;

public class ValkyrieTorqueHysteresisCompensator
{
   private final YoVariableRegistry registry = new YoVariableRegistry("TorqueHysteresisCompensator");

   private final List<OneDoFJoint> processedJoints = new ArrayList<>();
   private final List<TorqueHysteresisCompensatorYoVariable> hysteresisCompensators = new ArrayList<>();

   private final DoubleYoVariable torqueHysteresisAmplitude = new DoubleYoVariable("torqueHysteresisAmplitude", registry);
   private final DoubleYoVariable jointAccelerationMin = new DoubleYoVariable("hysteresisJointAccelerationMin", registry);
   private final DoubleYoVariable jointVelocityMax = new DoubleYoVariable("hysteresisJointVelocityMax", registry);

   private final DoubleYoVariable rampTime = new DoubleYoVariable("rampTime", registry);

   /**
    * Need to be extracted to a config file
    */
   private final String[] jointShortNamesToProcess = new String[]{"Hip", "Torso"};

   public ValkyrieTorqueHysteresisCompensator(RigidBody rootBody, DoubleYoVariable yoTime, YoVariableRegistry parentRegistry)
   {
      parentRegistry.addChild(registry);
      OneDoFJoint[] allJoints = ScrewTools.filterJoints(ScrewTools.computeSubtreeJoints(rootBody), OneDoFJoint.class);

      for (OneDoFJoint joint : allJoints)
      {
         String jointName = joint.getName();
         if (!shouldProcessJoint(jointName))
            continue;

         processedJoints.add(joint);
         hysteresisCompensators.add(new TorqueHysteresisCompensatorYoVariable("tau_hyst_", joint, torqueHysteresisAmplitude, jointAccelerationMin, jointVelocityMax, rampTime, yoTime, registry));
      }
   }

   public void compute()
   {
      for (int i = 0; i < processedJoints.size(); i++)
      {
         TorqueHysteresisCompensatorYoVariable hysteresisCompensator = hysteresisCompensators.get(i);
         hysteresisCompensator.update();
         processedJoints.get(i).setTau(hysteresisCompensator.getDoubleValue());
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
