package us.ihmc.avatar;

import java.util.Arrays;
import java.util.List;
import java.util.stream.Collectors;

import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.robotDataLogger.RobotVisualizer;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public class RobotVisualizerList implements RobotVisualizer
{
   private final List<RobotVisualizer> robotVisualizers;

   public RobotVisualizerList(RobotVisualizer... robotVisualizers)
   {
      this.robotVisualizers = Arrays.asList(robotVisualizers).stream().filter(v -> v != null).collect(Collectors.toList());
   }

   @Override
   public void update(long timestamp)
   {
      for (int i = 0; i < robotVisualizers.size(); i++)
      {
         robotVisualizers.get(i).update(timestamp);
      }
   }

   @Override
   public void update(long timestamp, YoVariableRegistry registry)
   {
      for (int i = 0; i < robotVisualizers.size(); i++)
      {
         robotVisualizers.get(i).update(timestamp, registry);
      }
   }

   @Override
   public void setMainRegistry(YoVariableRegistry registry, RigidBodyBasics rootBody, YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      for (int i = 0; i < robotVisualizers.size(); i++)
      {
         robotVisualizers.get(i).setMainRegistry(registry, rootBody, yoGraphicsListRegistry);
      }
   }

   @Override
   public void addRegistry(YoVariableRegistry registry, YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      for (int i = 0; i < robotVisualizers.size(); i++)
      {
         robotVisualizers.get(i).addRegistry(registry, yoGraphicsListRegistry);
      }
   }

   @Override
   public void close()
   {
      for (int i = 0; i < robotVisualizers.size(); i++)
      {
         robotVisualizers.get(i).close();
      }
   }

   @Override
   public long getLatestTimestamp()
   {
      long timestamp = -1;
      for (int i = 0; i < robotVisualizers.size(); i++)
      {
         long timestampLocal = robotVisualizers.get(i).getLatestTimestamp();
         if (timestamp != -1 && timestamp != timestampLocal)
         {
            throw new RuntimeException("Inconsistent timestamp");
         }
         timestamp = timestampLocal;
      }
      return timestamp;
   }

}
