package us.ihmc.sensorProcessing.outputData;

import java.util.HashMap;

import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.tools.lists.PairList;

public class JointDesiredOutputList implements JointDesiredOutputListReadOnly
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
}
