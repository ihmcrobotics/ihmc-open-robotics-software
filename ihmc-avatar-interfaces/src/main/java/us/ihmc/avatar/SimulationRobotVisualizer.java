package us.ihmc.avatar;

import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.simulationconstructionset.dataBuffer.MirroredYoVariableRegistry;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public class SimulationRobotVisualizer
{
   private final MirroredYoVariableRegistry registry;
   private final YoGraphicsListRegistry graphicsListRegistry;

   public SimulationRobotVisualizer(YoVariableRegistry registry, YoGraphicsListRegistry graphicsListRegistry)
   {
      this.graphicsListRegistry = graphicsListRegistry;
      this.registry = new MirroredYoVariableRegistry(registry);
   }

   public void update()
   {
      this.registry.updateMirror();
      if (graphicsListRegistry != null)
         graphicsListRegistry.update();
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
