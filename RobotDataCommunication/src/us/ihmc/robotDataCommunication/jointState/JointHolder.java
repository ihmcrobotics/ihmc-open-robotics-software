package us.ihmc.robotDataCommunication.jointState;

import us.ihmc.robotDataCommunication.generated.YoProtoHandshakeProto.YoProtoHandshake.JointDefinition.JointType;



public interface JointHolder
{
   public JointType getJointType();
   public int getNumberOfStateVariables();
   
   public void get(double[] buffer, int offset);
}
