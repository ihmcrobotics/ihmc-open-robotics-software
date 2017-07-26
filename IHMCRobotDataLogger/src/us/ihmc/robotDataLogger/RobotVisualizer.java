package us.ihmc.robotDataLogger;

import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public interface RobotVisualizer
{
   public void update(long timestamp);

   public void update(long timestamp, YoVariableRegistry registry);

   public void setMainRegistry(YoVariableRegistry registry, RigidBody rootBody, YoGraphicsListRegistry yoGraphicsListRegistry);

   public void addRegistry(YoVariableRegistry registry, YoGraphicsListRegistry yoGraphicsListRegistry);

   public void close();

   public long getLatestTimestamp();
}
