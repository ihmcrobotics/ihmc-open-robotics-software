package us.ihmc.simulationConstructionSetTools.robotController;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.robotController.SensorProcessor;

public class ModularSensorProcessor implements SensorProcessor
{
   private final ArrayList<SensorProcessor> sensorProcessors = new ArrayList<SensorProcessor>();
   private final String description;
   private final YoVariableRegistry registry;

   public ModularSensorProcessor(String name, String description, SensorProcessor sensorProcessor)
   {
      this(name, description);
      addSensorProcessor(sensorProcessor);
   }

   public ModularSensorProcessor(String name, String description, SensorProcessor[] sensorProcessors)
   {
      this(name, description);

      for (SensorProcessor sensorProcessor : sensorProcessors)
      {
         addSensorProcessor(sensorProcessor);
      }
   }

   public ModularSensorProcessor(String name, String description, List<SensorProcessor> sensorProcessors)
   {
      this(name, description);

      for (int i = 0; i < sensorProcessors.size(); i++)
      {
         addSensorProcessor(sensorProcessors.get(i));
      }
   }

   public ModularSensorProcessor(String name, String description)
   {
      this.description = description;
      this.registry = new YoVariableRegistry(name);
   }

   public void addSensorProcessor(SensorProcessor sensorProcessor)
   {
      this.sensorProcessors.add(sensorProcessor);
      this.registry.addChild(sensorProcessor.getYoVariableRegistry());
   }

   @Override
   public void initialize()
   {
      for (int i = 0; i < sensorProcessors.size(); i++)
      {
         sensorProcessors.get(i).initialize();
      }
   }

   @Override
   public void update()
   {
      for (int i = 0; i < sensorProcessors.size(); i++)
      {
         sensorProcessors.get(i).update();
      }
   }

   @Override
   public YoVariableRegistry getYoVariableRegistry()
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
      for (SensorProcessor sensorProcessor : sensorProcessors)
      {
         buf.append(sensorProcessor.getClass().getSimpleName() + "\n");
      }

      return buf.toString();
   }
}
