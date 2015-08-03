package us.ihmc.robotDataCommunication.jointState;

import us.ihmc.robotDataCommunication.generated.YoProtoHandshakeProto.YoProtoHandshake.JointDefinition.JointType;
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
