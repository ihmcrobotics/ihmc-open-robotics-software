package us.ihmc.sensorProcessing.outputData;

import gnu.trove.map.hash.TLongObjectHashMap;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.tools.lists.PairList;

public class JointDesiredOutputList implements JointDesiredOutputListBasics
{
   private final OneDoFJoint[] joints;
   private final JointDesiredOutput[] jointsData;
   private final TLongObjectHashMap<JointDesiredOutput> jointMap;

   public JointDesiredOutputList(OneDoFJoint[] joints)
   {
      this.joints = joints;
      this.jointsData = new JointDesiredOutput[joints.length];

      float disableAutoCompaction = 0;
      jointMap = new TLongObjectHashMap<>(joints.length);
      jointMap.setAutoCompactionFactor(disableAutoCompaction);

      for (int i = 0; i < joints.length; i++)
      {
         JointDesiredOutput data = new JointDesiredOutput();
         jointsData[i] = data;
         jointMap.put(joints[i].getNameBasedHashCode(), data);
      }


   }

   @Override
   public boolean hasDataForJoint(OneDoFJoint joint)
   {
      return jointMap.containsKey(joint.getNameBasedHashCode());
   }

   @Override
   public OneDoFJoint getOneDoFJoint(int index)
   {
      return joints[index];
   }

   @Override
   public JointDesiredOutput getJointDesiredOutput(OneDoFJoint joint)
   {
      return jointMap.get(joint.getNameBasedHashCode());
   }

   @Override
   public int getNumberOfJointsWithDesiredOutput()
   {
      return joints.length;
   }

   @Override
   public JointDesiredOutput getJointDesiredOutput(int index)
   {
      return jointsData[index];
   }

   public String getJointName(int index)
   {
      return joints[index].getName();
   }

   @Override
   public void clear()
   {
      for (JointDesiredOutput jointData : jointsData)
         jointData.clear();
   }

//   public void updateFromModel()
//   {
//
//      for (int i = 0; i < jointsAndData.size(); i++)
//      {
//
//         OneDoFJoint joint = jointsAndData.first(i);
//         LowLevelJointData data = jointsAndData.second(i);
//
//         data.setDesiredsFromOneDoFJoint(joint);
//      }
//   }

   @Override
   public void overwriteWith(JointDesiredOutputListReadOnly other)
   {
      for (int i = 0; i < joints.length; i++)
      {

         OneDoFJoint joint = joints[i];
         JointDesiredOutput data = jointsData[i];

         JointDesiredOutputReadOnly otherData = other.getJointDesiredOutput(joint);
         if (otherData != null)
         {
            data.set(otherData);
         }
      }
   }

   /**
    * Complete the information held in this using other.
    * Does not overwrite the data already set in this.
    */
   @Override
   public void completeWith(JointDesiredOutputListReadOnly other)
   {
      for (int i = 0; i < joints.length; i++)
      {
         OneDoFJoint joint = joints[i];
         JointDesiredOutput data = jointsData[i];

         JointDesiredOutputReadOnly otherData = other.getJointDesiredOutput(joint);
         if (otherData != null)
         {
            data.completeWith(otherData);
         }
      }
   }

   public void requestIntegratorReset()
   {
      for (JointDesiredOutput jointData : jointsData)
         jointData.setResetIntegrators(true);
   }
}
