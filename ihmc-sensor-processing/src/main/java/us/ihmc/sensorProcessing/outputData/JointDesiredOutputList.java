package us.ihmc.sensorProcessing.outputData;

import gnu.trove.map.hash.TIntObjectHashMap;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointReadOnly;

public class JointDesiredOutputList implements JointDesiredOutputListBasics
{
   private final OneDoFJointReadOnly[] joints;
   private final JointDesiredOutput[] jointsData;
   private final TIntObjectHashMap<JointDesiredOutput> jointMap;

   public JointDesiredOutputList(OneDoFJointReadOnly[] joints)
   {
      this.joints = joints;
      this.jointsData = new JointDesiredOutput[joints.length];

      float disableAutoCompaction = 0;
      jointMap = new TIntObjectHashMap<>(joints.length);
      jointMap.setAutoCompactionFactor(disableAutoCompaction);

      for (int i = 0; i < joints.length; i++)
      {
         JointDesiredOutput data = new JointDesiredOutput();
         jointsData[i] = data;
         jointMap.put(joints[i].hashCode(), data);
      }
   }

   @Override
   public boolean hasDataForJoint(OneDoFJointReadOnly joint)
   {
      return jointMap.containsKey(joint.hashCode());
   }

   @Override
   public OneDoFJointReadOnly getOneDoFJoint(int index)
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
   public JointDesiredOutput getJointDesiredOutput(OneDoFJointReadOnly joint)
   {
      return jointMap.get(joint.hashCode());
   }

   @Override
   public JointDesiredOutput getJointDesiredOutputFromHash(int jointHashCode)
   {
      return jointMap.get(jointHashCode);
   }

   public String getJointName(int index)
   {
      return joints[index].getName();
   }
}
