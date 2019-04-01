package us.ihmc.commonWalkingControlModules.controllerCore.command.lowLevel;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;

import gnu.trove.map.hash.TLongObjectHashMap;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutput;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputBasics;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputListBasics;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputListReadOnly;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputReadOnly;

public class LowLevelOneDoFJointDesiredDataHolder implements JointDesiredOutputListBasics
{
   private final List<OneDoFJointBasics> jointsWithDesiredData;
   private final RecyclingArrayList<JointDesiredOutput> lowLevelJointData;

   /**
    * This is used for lookups only and is populated from the {@link #lowLevelJointData}. It should not
    * be modified directly and does not represent the state of the class.
    */
   private final transient TLongObjectHashMap<JointDesiredOutput> lowLevelJointDataMap;

   public LowLevelOneDoFJointDesiredDataHolder()
   {
      this(50);
   }

   public LowLevelOneDoFJointDesiredDataHolder(int initialCapacity)
   {
      lowLevelJointData = new RecyclingArrayList<>(initialCapacity, JointDesiredOutput::new);

      jointsWithDesiredData = new ArrayList<>(initialCapacity);

      /**
       * A autoCompactionFactor of 0 disables auto-compacting, which ensures no garbage is created by
       * emptying and filling the map repeatedly. @dcalvert
       */
      float disableAutoCompaction = 0;
      lowLevelJointDataMap = new TLongObjectHashMap<>(initialCapacity);
      lowLevelJointDataMap.setAutoCompactionFactor(disableAutoCompaction);
   }

   @Override
   public void clear()
   {
      for (int i = 0; i < getNumberOfJointsWithDesiredOutput(); i++)
      {
         lowLevelJointData.get(i).clear();
      }
      jointsWithDesiredData.clear();
      lowLevelJointDataMap.clear();
      lowLevelJointData.clear();
   }

   public void registerJointsWithEmptyData(OneDoFJointBasics[] joints)
   {
      for (OneDoFJointBasics joint : joints)
         registerJointWithEmptyData(joint);
   }

   public JointDesiredOutput registerJointWithEmptyData(OneDoFJointBasics joint)
   {
      if (lowLevelJointDataMap.containsKey(joint.hashCode()))
         throwJointAlreadyRegisteredException(joint);

      return registerJointWithEmptyDataUnsafe(joint);
   }

   public void retrieveJointsFromName(Map<String, OneDoFJointBasics> nameToJointMap)
   {
      for (int i = 0; i < jointsWithDesiredData.size(); i++)
      {
         jointsWithDesiredData.set(i, nameToJointMap.get(jointsWithDesiredData.get(i).getName()));
      }
   }

   /**
    * Complete the information held in this using other. Does not overwrite the data already set in
    * this.
    */
   @Override
   public void completeWith(JointDesiredOutputListReadOnly other)
   {
      if (other == null)
         return;

      for (int i = 0; i < other.getNumberOfJointsWithDesiredOutput(); i++)
      {
         OneDoFJointBasics joint = other.getOneDoFJoint(i);

         JointDesiredOutputBasics lowLevelJointData = getJointDesiredOutputFromHash(joint.hashCode());
         JointDesiredOutputReadOnly otherLowLevelJointData = other.getJointDesiredOutput(i);

         if (lowLevelJointData != null)
         {
            lowLevelJointData.completeWith(otherLowLevelJointData);
         }
         else
         {
            registerLowLevelJointDataUnsafe(joint, otherLowLevelJointData);
         }
      }
   }

   /**
    * Clear this and copy the data held in other.
    */
   @Override
   public void overwriteWith(JointDesiredOutputListReadOnly other)
   {
      clear();

      if (other == null)
         return;

      for (int i = 0; i < other.getNumberOfJointsWithDesiredOutput(); i++)
      {
         OneDoFJointBasics joint = other.getOneDoFJoint(i);
         registerLowLevelJointDataUnsafe(joint, other.getJointDesiredOutput(i));
      }
   }

   @Override
   public boolean hasDataForJoint(OneDoFJointBasics joint)
   {
      return lowLevelJointDataMap.containsKey(joint.hashCode());
   }

   @Override
   public OneDoFJointBasics getOneDoFJoint(int index)
   {
      return jointsWithDesiredData.get(index);
   }

   @Override
   public int getNumberOfJointsWithDesiredOutput()
   {
      return jointsWithDesiredData.size();
   }

   @Override
   public JointDesiredOutputBasics getJointDesiredOutput(OneDoFJointBasics joint)
   {
      return lowLevelJointDataMap.get(joint.hashCode());
   }

   @Override
   public JointDesiredOutputBasics getJointDesiredOutputFromHash(int jointHashCode)
   {
      return lowLevelJointDataMap.get(jointHashCode);
   }

   @Override
   public JointDesiredOutputBasics getJointDesiredOutput(int index)
   {
      return lowLevelJointData.get(index);
   }

   private void registerLowLevelJointDataUnsafe(OneDoFJointBasics joint, JointDesiredOutputReadOnly jointDataToRegister)
   {
      JointDesiredOutput jointData = registerJointWithEmptyDataUnsafe(joint);
      jointData.set(jointDataToRegister);
   }

   private JointDesiredOutput registerJointWithEmptyDataUnsafe(OneDoFJointBasics joint)
   {
      JointDesiredOutput jointData = lowLevelJointData.add();
      lowLevelJointDataMap.put(joint.hashCode(), jointData);
      jointsWithDesiredData.add(joint);
      return jointData;
   }

   public void set(LowLevelOneDoFJointDesiredDataHolder other)
   {
      overwriteWith(other);
   }

   private static void throwJointAlreadyRegisteredException(OneDoFJointBasics joint)
   {
      throw new RuntimeException("The joint: " + joint.getName() + " has already been registered.");
   }

   @Override
   public boolean equals(Object object)
   {
      if (object instanceof JointDesiredOutputListReadOnly)
         return JointDesiredOutputListBasics.super.equals((JointDesiredOutputListReadOnly) object);
      else
         return false;
   }
}
