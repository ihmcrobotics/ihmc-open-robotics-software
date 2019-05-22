package us.ihmc.avatar;

import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.robotDataLogger.RobotVisualizer;
import us.ihmc.simulationconstructionset.dataBuffer.MirroredYoVariableRegistry;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public class SimulationRobotVisualizer implements RobotVisualizer
{
   private YoVariableRegistry originalRegistry;
   private MirroredYoVariableRegistry registry;
   private YoGraphicsListRegistry graphicsListRegistry;

   @Override
   public void update(long timestamp)
   {
      update(timestamp, registry);
   }

   @Override
   public void update(long timestamp, YoVariableRegistry registry)
   {
      if (registry != originalRegistry)
         throw new RuntimeException("Must call update with the original registry.");
      this.registry.updateMirror();
      if (graphicsListRegistry != null)
         graphicsListRegistry.update();
   }

   @Override
   public void setMainRegistry(YoVariableRegistry registry, RigidBodyBasics rootBody, YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      addRegistry(registry, yoGraphicsListRegistry);
   }

   @Override
   public void addRegistry(YoVariableRegistry registry, YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      if (this.registry != null)
         throw new RuntimeException("This implementation allows only one registry.");
      this.registry = new MirroredYoVariableRegistry(registry);
      this.graphicsListRegistry = yoGraphicsListRegistry;
      this.originalRegistry = registry;
   }

   @Override
   public void close()
   {
   }

   @Override
   public long getLatestTimestamp()
   {
      throw new UnsupportedOperationException();
   }

   public YoVariableRegistry getRegistry()
   {
      return registry;
   }

   public YoGraphicsListRegistry getGraphicsListRegistry()
   {
      return graphicsListRegistry;
   }

   public void updateGraphics()
   {
      registry.updateChangedValues();
      if (graphicsListRegistry != null)
         graphicsListRegistry.update();
   }
}
