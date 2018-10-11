package us.ihmc.commonWalkingControlModules.controllerCore.command.lowLevel;

import gnu.trove.map.hash.TLongObjectHashMap;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.sensorProcessing.outputData.*;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;

public class LowLevelOneDoFJointDesiredDataHolder implements JointDesiredOutputListBasics
{
   private final List<JointDesiredOutput> unusedData;
   private final List<OneDoFJoint> jointsWithDesiredData;
   private final TLongObjectHashMap<JointDesiredOutput> lowLevelJointDataMap;
   private final List<JointDesiredOutput> lowLevelJointData;

   public LowLevelOneDoFJointDesiredDataHolder()
   {
      this(50);
   }

   public LowLevelOneDoFJointDesiredDataHolder(int initialCapacity)
   {
      unusedData = new ArrayList<>(initialCapacity);
      lowLevelJointData = new ArrayList<>(initialCapacity);

      while (unusedData.size() < initialCapacity)
         unusedData.add(new JointDesiredOutput());

      jointsWithDesiredData = new ArrayList<>(initialCapacity);

      /**
       * A autoCompactionFactor of 0 disables auto-compacting, which ensures no garbage
       * is created by emptying and filling the map repeatedly. @dcalvert
       */
      float disableAutoCompaction = 0;
      lowLevelJointDataMap = new TLongObjectHashMap<JointDesiredOutput>(initialCapacity);
      lowLevelJointDataMap.setAutoCompactionFactor(disableAutoCompaction);
   }

   @Override
   public void clear()
   {
      lowLevelJointData.clear();
      while (!jointsWithDesiredData.isEmpty())
      {
         OneDoFJoint lastJoint = jointsWithDesiredData.remove(jointsWithDesiredData.size() - 1);
         JointDesiredOutput jointDataToReset = lowLevelJointDataMap.remove(lastJoint.getNameBasedHashCode());
         jointDataToReset.clear();
         unusedData.add(jointDataToReset);
      }
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
      JointDesiredOutput jointData;

      if (unusedData.isEmpty())
         jointData = new JointDesiredOutput();
      else
         jointData = unusedData.remove(unusedData.size() - 1);
      lowLevelJointDataMap.put(joint.getNameBasedHashCode(), jointData);
      jointsWithDesiredData.add(joint);
      lowLevelJointData.add(jointData);
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

         JointDesiredOutput lowLevelJointData = lowLevelJointDataMap.get(joint.getNameBasedHashCode());
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
   public JointDesiredOutputBasics getJointDesiredOutput(OneDoFJoint joint)
   {
      return lowLevelJointDataMap.get(joint.getNameBasedHashCode());
   }

   @Override
   public JointDesiredOutputBasics getJointDesiredOutput(long jointName)
   {
      return lowLevelJointDataMap.get(jointName);
   }

   @Override
   public JointDesiredOutputBasics getJointDesiredOutput(int index)
   {
      return lowLevelJointData.get(index);
   }


   private static void throwJointAlreadyRegisteredException(OneDoFJoint joint)
   {
      throw new RuntimeException("The joint: " + joint.getName() + " has already been registered.");
   }
}
