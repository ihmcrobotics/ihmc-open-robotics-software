package us.ihmc.sensorProcessing.outputData;

import java.util.HashMap;

import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.tools.lists.PairList;

public class JointDesiredOutputList implements JointDesiredOutputListBasics
{
   private final PairList<OneDoFJoint, JointDesiredOutput> jointsAndData = new PairList<>();
   private final HashMap<OneDoFJoint, JointDesiredOutput> jointMap = new HashMap<>();

   public JointDesiredOutputList(OneDoFJoint[] joints)
   {
      for (OneDoFJoint joint : joints)
      {
         JointDesiredOutput data = new JointDesiredOutput();
         jointsAndData.add(joint, data);
         jointMap.put(joint, data);
      }
   }

   @Override
   public boolean hasDataForJoint(OneDoFJoint joint)
   {
      return jointMap.containsKey(joint);
   }

   @Override
   public OneDoFJoint getOneDoFJoint(int index)
   {
      return jointsAndData.first(index);
   }

   @Override
   public JointDesiredOutput getJointDesiredOutput(OneDoFJoint joint)
   {
      return jointMap.get(joint);
   }

   @Override
   public int getNumberOfJointsWithDesiredOutput()
   {
      return jointsAndData.size();
   }

   @Override
   public JointDesiredOutput getJointDesiredOutput(int index)
   {
      return jointsAndData.second(index);
   }

   public String getJointName(int index)
   {
      return jointsAndData.first(index).getName();
   }

   @Override
   public void clear()
   {
      for (int jointIdx = 0; jointIdx < jointsAndData.size(); jointIdx++)
      {
         jointsAndData.get(jointIdx).getValue().clear();
      }
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
      for (int i = 0; i < jointsAndData.size(); i++)
      {

         OneDoFJoint joint = jointsAndData.first(i);
         JointDesiredOutput data = jointsAndData.second(i);

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
      for (int i = 0; i < jointsAndData.size(); i++)
      {

         OneDoFJoint joint = jointsAndData.first(i);
         JointDesiredOutput data = jointsAndData.second(i);

         JointDesiredOutputReadOnly otherData = other.getJointDesiredOutput(joint);
         if (otherData != null)
         {
            data.completeWith(otherData);
         }
      }
   }

   public void requestIntegratorReset()
   {
      for (int jointIdx = 0; jointIdx < jointsAndData.size(); jointIdx++)
      {
         jointsAndData.get(jointIdx).getValue().setResetIntegrators(true);
      }
   }
}
