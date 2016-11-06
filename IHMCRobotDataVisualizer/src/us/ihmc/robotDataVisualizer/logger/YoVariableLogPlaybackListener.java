package us.ihmc.robotDataVisualizer.logger;

import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.simulationconstructionset.FloatingRootJointRobot;

public interface YoVariableLogPlaybackListener
{
   public void setRobot(FloatingRootJointRobot robot);
   public void setYoVariableRegistry(YoVariableRegistry registry);
   
   public void updated(long timestamp);
}
