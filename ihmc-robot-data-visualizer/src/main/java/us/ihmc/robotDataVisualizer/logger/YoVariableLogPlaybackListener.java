package us.ihmc.robotDataVisualizer.logger;

import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.simulationconstructionset.FloatingRootJointRobot;

public interface YoVariableLogPlaybackListener
{
   public void setRobot(FloatingRootJointRobot robot);
   public void setYoVariableRegistry(YoRegistry registry);
   
   public void updated(long timestamp);
}
