package us.ihmc.commonWalkingControlModules.momentumBasedController.optimization;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import org.apache.commons.lang3.mutable.MutableDouble;

import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.screwTheory.InverseDynamicsJoint;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.robotics.screwTheory.ScrewTools;

public class DesiredOneDoFJointTorqueHolder
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final List<MutableDouble> unusedMutableDoubles;
   private final List<OneDoFJoint> jointsWithDesiredTorques;
   private final Map<OneDoFJoint, MutableDouble> jointDesiredTorqueMap;
   private final Map<OneDoFJoint, DoubleYoVariable> yoJointDesiredTorqueMap;

   public DesiredOneDoFJointTorqueHolder(InverseDynamicsJoint[] allJoints, YoVariableRegistry parentRegistry)
   {
      OneDoFJoint[] allOneDoFJoints = ScrewTools.filterJoints(allJoints, OneDoFJoint.class);
      int numberOfOneDoFJoints = allOneDoFJoints.length;
      unusedMutableDoubles = new ArrayList<>(numberOfOneDoFJoints);
      jointsWithDesiredTorques = new ArrayList<>(numberOfOneDoFJoints);
      jointDesiredTorqueMap = new HashMap<>(numberOfOneDoFJoints);
      yoJointDesiredTorqueMap = new HashMap<>(numberOfOneDoFJoints);

      for (int i = 0; i < numberOfOneDoFJoints; i++)
      {
         OneDoFJoint oneDoFJoint = allOneDoFJoints[i];
         DoubleYoVariable yoJointDesiredTorque = new DoubleYoVariable("tau_d_" + oneDoFJoint.getName(), registry);
         yoJointDesiredTorqueMap.put(oneDoFJoint, yoJointDesiredTorque);
      }

      parentRegistry.addChild(registry);
   }

   public void reset()
   {
      for (int i = 0; i < jointsWithDesiredTorques.size(); i++)
         unusedMutableDoubles.add(jointDesiredTorqueMap.remove(jointsWithDesiredTorques.get(i)));
      jointsWithDesiredTorques.clear();
   }

   public int getNumberOfJoints()
   {
      return jointsWithDesiredTorques.size();
   }

   public void extractDesiredTorquesFromInverseDynamicsJoints(InverseDynamicsJoint[] inverseDynamicsJoints)
   {
      for (int i = 0; i < inverseDynamicsJoints.length; i++)
      {
         if (inverseDynamicsJoints[i] instanceof OneDoFJoint)
         {
            OneDoFJoint oneDoFJoint = (OneDoFJoint) inverseDynamicsJoints[i];
            registerDesiredTorque(oneDoFJoint, oneDoFJoint.getTau());
         }
      }
   }

   private void registerDesiredTorque(OneDoFJoint oneDoFJoint, double tauDesired)
   {
      jointsWithDesiredTorques.add(oneDoFJoint);

      if (jointDesiredTorqueMap.containsKey(oneDoFJoint))
         throw new RuntimeException("Reset before registering new joint torques.");

      MutableDouble jointMutableTorque;

      if (unusedMutableDoubles.isEmpty())
         jointMutableTorque = new MutableDouble();
      else
         jointMutableTorque = unusedMutableDoubles.remove(unusedMutableDoubles.size() - 1);
      jointDesiredTorqueMap.put(oneDoFJoint, jointMutableTorque);
      jointMutableTorque.setValue(tauDesired);

      yoJointDesiredTorqueMap.get(oneDoFJoint).set(tauDesired);
   }

   public OneDoFJoint getOneDoFJoint(int index)
   {
      return jointsWithDesiredTorques.get(index);
   }

   public double getDesiredJointTorque(int index)
   {
      return jointDesiredTorqueMap.get(jointsWithDesiredTorques.get(index)).doubleValue();
   }

   public void set(DesiredOneDoFJointTorqueHolder other)
   {
      reset();
      for (int i = 0; i < other.getNumberOfJoints(); i++)
      {
         OneDoFJoint oneDoFJoint = other.getOneDoFJoint(i);
         double desiredJointTorque = other.getDesiredJointTorque(i);
         registerDesiredTorque(oneDoFJoint, desiredJointTorque);
      }
   }
}
