package us.ihmc.commonWalkingControlModules.momentumBasedController.optimization;

import java.util.HashMap;
import java.util.Map;

import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.screwTheory.InverseDynamicsJoint;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.robotics.screwTheory.ScrewTools;

public class YoDesiredOneDoFJointTorqueHolder extends DesiredOneDoFJointTorqueHolder
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final Map<OneDoFJoint, DoubleYoVariable> yoJointDesiredTorqueMap;

   public YoDesiredOneDoFJointTorqueHolder(InverseDynamicsJoint[] allJoints, YoVariableRegistry parentRegistry)
   {
      super(allJoints.length);
      OneDoFJoint[] allOneDoFJoints = ScrewTools.filterJoints(allJoints, OneDoFJoint.class);
      int numberOfOneDoFJoints = allOneDoFJoints.length;
      yoJointDesiredTorqueMap = new HashMap<>(numberOfOneDoFJoints);

      for (int i = 0; i < numberOfOneDoFJoints; i++)
      {
         OneDoFJoint oneDoFJoint = allOneDoFJoints[i];
         DoubleYoVariable yoJointDesiredTorque = new DoubleYoVariable("tau_d_" + oneDoFJoint.getName(), registry);
         yoJointDesiredTorqueMap.put(oneDoFJoint, yoJointDesiredTorque);
      }

      parentRegistry.addChild(registry);
   }

   @Override
   public void registerDesiredTorque(OneDoFJoint oneDoFJoint, double tauDesired)
   {
      super.registerDesiredTorque(oneDoFJoint, tauDesired);
      yoJointDesiredTorqueMap.get(oneDoFJoint).set(tauDesired);
   }
}
