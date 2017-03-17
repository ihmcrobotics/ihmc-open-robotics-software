package us.ihmc.robotDataLogger.jointState;

import us.ihmc.idl.us.ihmc.robotDataLogger.JointType;
import us.ihmc.robotics.screwTheory.OneDoFJoint;

public class OneDoFJointHolder implements JointHolder
{
   private final OneDoFJoint joint;
   
   public OneDoFJointHolder(OneDoFJoint joint)
   {
      this.joint = joint;
   }

   public JointType getJointType()
   {
      return JointType.OneDoFJoint;
   }

   public int getNumberOfStateVariables()
   {
      return 2;
   }

   public void get(double[] buffer, int offset)
   {
      buffer[offset + 0] = joint.getQ();
      buffer[offset + 1] = joint.getQd();
   }

}
