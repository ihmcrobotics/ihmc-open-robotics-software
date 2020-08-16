package us.ihmc.simulationConstructionSetTools.robotController;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.robotics.robotController.RawSensorReader;

public class ModularRawSensorReader implements RawSensorReader
{
   private final ArrayList<RawSensorReader> rawSensorReaders = new ArrayList<RawSensorReader>();
   private final String description;
   private final YoRegistry registry;

   public ModularRawSensorReader(String name, String description, RawSensorReader rawSensorReader)
   {
      this(name, description);
      addRawSensorReader(rawSensorReader);
   }

   public ModularRawSensorReader(String name, String description, RawSensorReader[] rawSensorReaders)
   {
      this(name, description);

      for (RawSensorReader rawSensorReader : rawSensorReaders)
      {
         addRawSensorReader(rawSensorReader);
      }
   }

   public ModularRawSensorReader(String name, String description, List<RawSensorReader> rawSensorReaders)
   {
      this(name, description);

      for (RawSensorReader rawSensorReader : rawSensorReaders)
      {
         addRawSensorReader(rawSensorReader);
      }
   }

   public ModularRawSensorReader(String name, String description)
   {
      this.description = description;
      this.registry = new YoRegistry(name);
   }

   public void addRawSensorReader(RawSensorReader rawSensorReader)
   {
      this.rawSensorReaders.add(rawSensorReader);
      this.registry.addChild(rawSensorReader.getYoRegistry());
   }

   @Override
   public void initialize()
   {
      for (RawSensorReader rawSensorReader : rawSensorReaders)
         rawSensorReader.initialize();
   }

   @Override
   public void read()
   {
      for (RawSensorReader rawSensorReader : rawSensorReaders)
         rawSensorReader.read();
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
      for (RawSensorReader rawSensorReader : rawSensorReaders)
      {
         buf.append(rawSensorReader.getClass().getSimpleName() + "\n");
      }
      return buf.toString();
   }
}
