package us.ihmc.simulationConstructionSetTools.robotController;

import java.util.ArrayList;

import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.robotics.robotController.OutputProcessor;

public class ModularOutputProcessor implements OutputProcessor
{
   private final ArrayList<OutputProcessor> outputProcessors = new ArrayList<OutputProcessor>();
   private final String description;
   private final YoRegistry registry;

   public ModularOutputProcessor(String name, String description, OutputProcessor outputProcessor)
   {
      this(name, description);
      addOutputProcessor(outputProcessor);
   }

   public ModularOutputProcessor(String name, String description, OutputProcessor[] outputProcessors)
   {
      this(name, description);

      for (OutputProcessor outputProcessor : outputProcessors)
      {
         addOutputProcessor(outputProcessor);
      }
   }

   public ModularOutputProcessor(String name, String description, ArrayList<OutputProcessor> outputProcessors)
   {
      this(name, description);

      for (OutputProcessor outputProcessor : outputProcessors)
      {
         addOutputProcessor(outputProcessor);
      }
   }

   public ModularOutputProcessor(String name, String description)
   {
      this.description = description;
      this.registry = new YoRegistry(name);
   }

   public void addOutputProcessor(OutputProcessor outputProcessor)
   {
      this.outputProcessors.add(outputProcessor);
      this.registry.addChild(outputProcessor.getYoRegistry());
   }

   @Override
   public void initialize()
   {
      for (int i = 0; i < outputProcessors.size(); i++)
      {
         outputProcessors.get(i).initialize();
      }
   }

   @Override
   public void update()
   {
      for (int i = 0; i < outputProcessors.size(); i++)
      {
         outputProcessors.get(i).update();
      }
   }

   @Override
   public YoRegistry getYoRegistry()
   {
      return registry;
   }

   @Override
   public String getName()
   {
      return registry.getName();
   }

   @Override
   public String getDescription()
   {
      return description;
   }

   @Override
   public String toString()
   {
      StringBuffer buf = new StringBuffer();
      for (OutputProcessor outputProcessor : outputProcessors)
      {
         buf.append(outputProcessor.getClass().getSimpleName() + "\n");
      }
      return buf.toString();
   }
}
