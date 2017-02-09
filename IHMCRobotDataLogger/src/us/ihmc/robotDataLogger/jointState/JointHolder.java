package us.ihmc.robotDataLogger.jointState;

import us.ihmc.robotDataLogger.generated.YoProtoHandshakeProto.YoProtoHandshake.JointDefinition.JointType;



public interface JointHolder
{
   public JointType getJointType();
   public int getNumberOfStateVariables();
   
   public void get(double[] buffer, int offset);
}
