package us.ihmc.sensorProcessing.outputData;

import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.tools.lists.PairList;

import java.util.HashMap;

public class LowLevelStateList implements LowLevelStateListReadOnly
{
   private final PairList<OneDoFJoint, LowLevelState> jointsAndData = new PairList<>();
   private final HashMap<OneDoFJoint, LowLevelState> jointMap = new HashMap<>();

   public LowLevelStateList(OneDoFJoint[] joints)
   {
      for (OneDoFJoint joint : joints)
      {
         LowLevelState data = new LowLevelState();
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
   public LowLevelState getLowLevelState(OneDoFJoint joint)
   {
      return jointMap.get(joint);
   }

   @Override
   public int getNumberOfJointsWithLowLevelState()
   {
      return jointsAndData.size();
   }

   @Override
   public LowLevelStateReadOnly getLowLevelState(int index)
   {
      return jointsAndData.second(index);
   }

   public String getJointName(int index)
   {
      return jointsAndData.first(index).getName();
   }

   public void clear()
   {
      for (int jointIdx = 0; jointIdx < jointsAndData.size(); jointIdx++)
      {
         jointsAndData.get(jointIdx).getValue().clear();
      }
   }

   public void overwriteWith(LowLevelStateListReadOnly other)
   {
      for (int i = 0; i < jointsAndData.size(); i++)
      {

         OneDoFJoint joint = jointsAndData.first(i);
         LowLevelState data = jointsAndData.second(i);

         LowLevelStateReadOnly otherData = other.getLowLevelState(joint);
         if (otherData != null)
         {
            data.set(otherData);
         }

      }
   }
}
