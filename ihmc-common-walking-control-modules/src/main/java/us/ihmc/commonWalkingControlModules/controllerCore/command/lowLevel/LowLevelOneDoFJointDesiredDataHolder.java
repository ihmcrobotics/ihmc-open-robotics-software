package us.ihmc.commonWalkingControlModules.controllerCore.command.lowLevel;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;

import gnu.trove.map.hash.TLongObjectHashMap;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.sensorProcessing.outputData.*;

public class LowLevelOneDoFJointDesiredDataHolder implements JointDesiredOutputListBasics
{
   private final List<OneDoFJoint> jointsWithDesiredData;
   private final RecyclingArrayList<JointDesiredOutput> lowLevelJointData;
   private final TLongObjectHashMap<JointDesiredOutput> lowLevelJointDataMap;

   public LowLevelOneDoFJointDesiredDataHolder()
   {
      this(50);
   }

   public LowLevelOneDoFJointDesiredDataHolder(int initialCapacity)
   {
      lowLevelJointData = new RecyclingArrayList<>(initialCapacity, JointDesiredOutput::new);
      lowLevelJointData.clear();

      jointsWithDesiredData = new ArrayList<>(initialCapacity);

      /**
       * A autoCompactionFactor of 0 disables auto-compacting, which ensures no garbage
       * is created by emptying and filling the map repeatedly. @dcalvert
       */
      float disableAutoCompaction = 0;
      lowLevelJointDataMap = new TLongObjectHashMap<>(initialCapacity);
      lowLevelJointDataMap.setAutoCompactionFactor(disableAutoCompaction);
   }

   @Override
   public void clear()
   {
      while (!jointsWithDesiredData.isEmpty())
      {
         OneDoFJoint lastJoint = jointsWithDesiredData.remove(jointsWithDesiredData.size() - 1);
         JointDesiredOutput jointDataToReset = lowLevelJointDataMap.remove(lastJoint.getNameBasedHashCode());
         jointDataToReset.clear();
      }

      lowLevelJointData.clear();
   }

   public void registerJointsWithEmptyData(OneDoFJoint[] joint)
   {
      for (int i = 0; i < joint.length; i++)
         registerJointWithEmptyData(joint[i]);
   }

   public JointDesiredOutput registerJointWithEmptyData(OneDoFJoint joint)
   {
      if (lowLevelJointDataMap.containsKey(joint.getNameBasedHashCode()))
         throwJointAlreadyRegisteredException(joint);

      return registerJointWithEmptyDataUnsafe(joint);
   }

   public void registerLowLevelJointData(OneDoFJoint joint, JointDesiredOutputReadOnly jointDataToRegister)
   {
      if (lowLevelJointDataMap.containsKey(joint.getNameBasedHashCode()))
         throwJointAlreadyRegisteredException(joint);

      registerLowLevelJointDataUnsafe(joint, jointDataToRegister);
   }

   private void registerLowLevelJointDataUnsafe(OneDoFJoint joint, JointDesiredOutputReadOnly jointDataToRegister)
   {
      JointDesiredOutput jointData = registerJointWithEmptyDataUnsafe(joint);
      jointData.set(jointDataToRegister);
   }

   private JointDesiredOutput registerJointWithEmptyDataUnsafe(OneDoFJoint joint)
   {
      JointDesiredOutput jointData = lowLevelJointData.add();
      jointsWithDesiredData.add(joint);
      lowLevelJointDataMap.put(joint.getNameBasedHashCode(), jointData);
      return jointData;
   }

   public void retrieveJointsFromName(Map<String, OneDoFJoint> nameToJointMap)
   {
      for (int i = 0; i < jointsWithDesiredData.size(); i++)
      {
         jointsWithDesiredData.set(i, nameToJointMap.get(jointsWithDesiredData.get(i).getName()));
      }
   }





   /**
    * Complete the information held in this using other.
    * Does not overwrite the data already set in this.
    */
   @Override
   public void completeWith(JointDesiredOutputListReadOnly other)
   {
      if (other == null)
         return;

      for (int i = 0; i < other.getNumberOfJointsWithDesiredOutput(); i++)
      {
         OneDoFJoint joint = other.getOneDoFJoint(i);

         JointDesiredOutputBasics lowLevelJointData = getJointDesiredOutput(joint.getNameBasedHashCode());
         JointDesiredOutputReadOnly otherLowLevelJointData = other.getJointDesiredOutput(joint);

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
         OneDoFJoint joint = other.getOneDoFJoint(i);
         registerLowLevelJointDataUnsafe(joint, other.getJointDesiredOutput(joint));
      }
   }



   @Override
   public boolean hasDataForJoint(OneDoFJoint joint)
   {
      return lowLevelJointDataMap.containsKey(joint.getNameBasedHashCode());
   }


   static void throwJointAlreadyRegisteredException(OneDoFJoint joint)
   {
      throw new RuntimeException("The joint: " + joint.getName() + " has already been registered.");
   }

   @Override
   public OneDoFJoint getOneDoFJoint(int index)
   {
      return jointsWithDesiredData.get(index);
   }

   @Override
   public int getNumberOfJointsWithDesiredOutput()
   {
      return jointsWithDesiredData.size();
   }

   @Override
   public JointDesiredOutput getJointDesiredOutput(OneDoFJoint joint)
   {
      return getJointDesiredOutput(joint.getNameBasedHashCode());
   }

   @Override
   public JointDesiredOutput getJointDesiredOutput(long jointName)
   {
      return lowLevelJointDataMap.get(jointName);
   }

   @Override
   public JointDesiredOutputBasics getJointDesiredOutput(int index)
   {
      return lowLevelJointData.get(index);
   }


}
