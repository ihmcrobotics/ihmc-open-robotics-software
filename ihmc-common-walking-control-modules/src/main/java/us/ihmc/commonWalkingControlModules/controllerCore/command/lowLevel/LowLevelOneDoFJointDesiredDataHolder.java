package us.ihmc.commonWalkingControlModules.controllerCore.command.lowLevel;

import java.util.ArrayList;
import java.util.List;

import gnu.trove.map.hash.TLongObjectHashMap;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutput;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputBasics;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputListBasics;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputListReadOnly;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputReadOnly;

public class LowLevelOneDoFJointDesiredDataHolder implements JointDesiredOutputListBasics
{
   private final List<OneDoFJointBasics> joints = new ArrayList<>();
   private final List<JointDesiredOutput> jointOutputs = new ArrayList<>();

   /**
    * This map guarantees that the same output object will be reused for the same joint even if the joint is removed
    * from this data holder and later added again. It serves as storage of the output objects so they can be recycled
    * and does not represent the state of this data holder.
    */
   private final transient TLongObjectHashMap<JointDesiredOutput> lowLevelJointDataMap;

   public LowLevelOneDoFJointDesiredDataHolder()
   {
      this(50);
   }

   public LowLevelOneDoFJointDesiredDataHolder(OneDoFJointBasics[] joints)
   {
      this(joints.length);
      registerJointsWithEmptyData(joints);
   }

   public LowLevelOneDoFJointDesiredDataHolder(int initialCapacity)
   {
      float disableAutoCompaction = 0;
      lowLevelJointDataMap = new TLongObjectHashMap<>(initialCapacity);
      lowLevelJointDataMap.setAutoCompactionFactor(disableAutoCompaction);
   }

   @Override
   public void clear()
   {
      joints.clear();
      jointOutputs.clear();
   }

   public void registerJointsWithEmptyData(OneDoFJointBasics[] joints)
   {
      for (OneDoFJointBasics joint : joints)
         registerJointWithEmptyData(joint);
   }

   public JointDesiredOutput registerJointWithEmptyData(OneDoFJointBasics joint)
   {
      if (joints.contains(joint))
         JointDesiredOutputListBasics.throwJointAlreadyRegisteredException(joint);
      return registerJointWithEmptyDataUnsafe(joint);
   }

   private void registerLowLevelJointDataUnsafe(OneDoFJointBasics joint, JointDesiredOutputReadOnly jointDataToRegister)
   {
      JointDesiredOutput jointData = registerJointWithEmptyDataUnsafe(joint);
      jointData.set(jointDataToRegister);
   }

   private JointDesiredOutput registerJointWithEmptyDataUnsafe(OneDoFJointBasics joint)
   {
      JointDesiredOutput jointData = lowLevelJointDataMap.get(joint.hashCode());
      if (jointData == null)
      {
         jointData = new JointDesiredOutput();
         lowLevelJointDataMap.put(joint.hashCode(), jointData);
      }
      joints.add(joint);
      jointOutputs.add(jointData);
      return jointData;
   }

   @Override
   public boolean hasDataForJoint(OneDoFJointBasics joint)
   {
      return joints.contains(joint);
   }

   @Override
   public OneDoFJointBasics getOneDoFJoint(int index)
   {
      return joints.get(index);
   }

   @Override
   public int getNumberOfJointsWithDesiredOutput()
   {
      return joints.size();
   }

   @Override
   public JointDesiredOutputBasics getJointDesiredOutputFromHash(int jointHashCode)
   {
      return lowLevelJointDataMap.get(jointHashCode);
   }

   @Override
   public JointDesiredOutputBasics getJointDesiredOutput(int index)
   {
      return jointOutputs.get(index);
   }

   @Override
   public void completeWith(JointDesiredOutputListReadOnly other)
   {
      if (other == null)
         return;

      for (int i = 0; i < other.getNumberOfJointsWithDesiredOutput(); i++)
      {
         OneDoFJointBasics joint = other.getOneDoFJoint(i);
         JointDesiredOutputReadOnly otherLowLevelJointData = other.getJointDesiredOutput(i);
         if (hasDataForJoint(joint))
         {
            getJointDesiredOutput(joint).completeWith(otherLowLevelJointData);
         }
         else
         {
            registerLowLevelJointDataUnsafe(joint, otherLowLevelJointData);
         }
      }
   }

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

   public void set(LowLevelOneDoFJointDesiredDataHolder other)
   {
      overwriteWith(other);
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
