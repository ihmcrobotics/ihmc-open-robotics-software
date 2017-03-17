package us.ihmc.robotDataLogger.jointState;

import us.ihmc.idl.us.ihmc.robotDataLogger.JointType;

public interface JointHolder
{
   public JointType getJointType();
   public int getNumberOfStateVariables();
   
   public void get(double[] buffer, int offset);
}
