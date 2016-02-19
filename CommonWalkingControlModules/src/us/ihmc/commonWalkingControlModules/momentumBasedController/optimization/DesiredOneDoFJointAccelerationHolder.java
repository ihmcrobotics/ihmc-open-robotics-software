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

public class DesiredOneDoFJointAccelerationHolder
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final List<MutableDouble> unusedMutableDoubles;
   private final List<OneDoFJoint> jointsWithDesiredAccelerations;
   private final Map<OneDoFJoint, MutableDouble> jointDesiredAccelerationMap;
   private final Map<OneDoFJoint, DoubleYoVariable> yoJointDesiredAccelerationMap;

   public DesiredOneDoFJointAccelerationHolder(InverseDynamicsJoint[] allJoints, YoVariableRegistry parentRegistry)
   {
      OneDoFJoint[] allOneDoFJoints = ScrewTools.filterJoints(allJoints, OneDoFJoint.class);
      int numberOfOneDoFJoints = allOneDoFJoints.length;
      unusedMutableDoubles = new ArrayList<>(numberOfOneDoFJoints);
      jointsWithDesiredAccelerations = new ArrayList<>(numberOfOneDoFJoints);
      jointDesiredAccelerationMap = new HashMap<>(numberOfOneDoFJoints);
      yoJointDesiredAccelerationMap = new HashMap<>(numberOfOneDoFJoints);

      for (int i = 0; i < numberOfOneDoFJoints; i++)
      {
         OneDoFJoint oneDoFJoint = allOneDoFJoints[i];
         DoubleYoVariable yoJointDesiredAcceleration = new DoubleYoVariable("qdd_d_" + oneDoFJoint.getName(), registry);
         yoJointDesiredAccelerationMap.put(oneDoFJoint, yoJointDesiredAcceleration);
      }

      parentRegistry.addChild(registry);
   }

   public void reset()
   {
      for (int i = 0; i < jointsWithDesiredAccelerations.size(); i++)
         unusedMutableDoubles.add(jointDesiredAccelerationMap.remove(jointsWithDesiredAccelerations.get(i)));
      jointsWithDesiredAccelerations.clear();
   }

   public int getNumberOfJoints()
   {
      return jointsWithDesiredAccelerations.size();
   }

   public void extractDesiredAccelerationsFromInverseDynamicsJoints(InverseDynamicsJoint[] inverseDynamicsJoints)
   {
      for (int i = 0; i < inverseDynamicsJoints.length; i++)
      {
         if (inverseDynamicsJoints[i] instanceof OneDoFJoint)
         {
            OneDoFJoint oneDoFJoint = (OneDoFJoint) inverseDynamicsJoints[i];
            registerDesiredAcceleration(oneDoFJoint, oneDoFJoint.getQddDesired());
         }
      }
   }

   private void registerDesiredAcceleration(OneDoFJoint oneDoFJoint, double accelerationDesired)
   {
      jointsWithDesiredAccelerations.add(oneDoFJoint);

      if (jointDesiredAccelerationMap.containsKey(oneDoFJoint))
         throw new RuntimeException("Reset before registering new joint torques.");

      MutableDouble jointMutableAcceleration;

      if (unusedMutableDoubles.isEmpty())
         jointMutableAcceleration = new MutableDouble();
      else
         jointMutableAcceleration = unusedMutableDoubles.remove(unusedMutableDoubles.size() - 1);
      jointDesiredAccelerationMap.put(oneDoFJoint, jointMutableAcceleration);
      jointMutableAcceleration.setValue(accelerationDesired);

      yoJointDesiredAccelerationMap.get(oneDoFJoint).set(accelerationDesired);
   }

   public OneDoFJoint getOneDoFJoint(int index)
   {
      return jointsWithDesiredAccelerations.get(index);
   }

   public double getDesiredJointAcceleration(int index)
   {
      return jointDesiredAccelerationMap.get(jointsWithDesiredAccelerations.get(index)).doubleValue();
   }

   public void set(DesiredOneDoFJointAccelerationHolder other)
   {
      reset();
      for (int i = 0; i < other.getNumberOfJoints(); i++)
      {
         OneDoFJoint oneDoFJoint = other.getOneDoFJoint(i);
         double desiredJointAcceleration = other.getDesiredJointAcceleration(i);
         registerDesiredAcceleration(oneDoFJoint, desiredJointAcceleration);
      }
   }
}
