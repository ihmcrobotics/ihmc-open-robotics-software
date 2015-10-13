package us.ihmc.robotDataCommunication.logger;

import us.ihmc.SdfLoader.SDFHumanoidRobot;

public interface YoVariableLogPlaybackListener
{
   public void setRobot(SDFHumanoidRobot robot);
   public void updated(long timestamp);
}
