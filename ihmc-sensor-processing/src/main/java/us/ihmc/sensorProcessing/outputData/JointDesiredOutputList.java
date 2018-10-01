package us.ihmc.sensorProcessing.outputData;

import java.util.HashMap;
import java.util.Map;

import gnu.trove.map.hash.TLongObjectHashMap;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.tools.lists.PairList;

public class JointDesiredOutputList implements JointDesiredOutputListBasics
{
   private final int numberOfJoints;
   private final OneDoFJoint[] joints;
   private final JointDesiredOutput[] jointData;
   private final TLongObjectHashMap<JointDesiredOutput> jointMap;

   public JointDesiredOutputList(OneDoFJoint[] joints)
   {
      this.numberOfJoints = joints.length;
      this.joints = joints;
      this.jointData = new JointDesiredOutput[joints.length];

      float disableAutoCompaction = 0;
      jointMap = new TLongObjectHashMap<>(numberOfJoints);
      jointMap.setAutoCompactionFactor(disableAutoCompaction);
      for (int i = 0; i < joints.length; i++)
      {
         jointData[i] = new JointDesiredOutput();
         jointMap.put(joints[i].getNameBasedHashCode(), jointData[i]);
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
   public int getNumberOfJointsWithDesiredOutput()
   {
      return numberOfJoints;
   }

   @Override
   public JointDesiredOutput getJointDesiredOutput(OneDoFJoint joint)
   {
      return getJointDesiredOutput(joint.getNameBasedHashCode());
   }

   @Override
   public JointDesiredOutput getJointDesiredOutput(long jointName)
   {
      return jointMap.get(jointName);
   }

   @Override
   public JointDesiredOutput getJointDesiredOutput(int index)
   {
      return jointData[index];
   }

   public String getJointName(int index)
   {
      return joints[index].getName();
   }

}
