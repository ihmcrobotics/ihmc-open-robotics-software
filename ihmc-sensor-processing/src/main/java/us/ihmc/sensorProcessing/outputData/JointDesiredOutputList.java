package us.ihmc.sensorProcessing.outputData;

import gnu.trove.map.hash.TLongObjectHashMap;
import us.ihmc.robotics.screwTheory.OneDoFJoint;

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
   public int getNumberOfJointsWithDesiredOutput()
   {
      return joints.length;
   }

   @Override
   public JointDesiredOutput getJointDesiredOutput(int index)
   {
      return jointsData[index];
   }

   @Override
   public JointDesiredOutput getJointDesiredOutput(OneDoFJoint joint)
   {
      return jointMap.get(joint.getNameBasedHashCode());
   }

   @Override
   public JointDesiredOutput getJointDesiredOutput(long jointName)
   {
      return jointMap.get(jointName);
   }

   public String getJointName(int index)
   {
      return joints[index].getName();
   }
}
