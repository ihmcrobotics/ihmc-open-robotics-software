package us.ihmc.simulationConstructionSetTools.robotController;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.robotController.RawOutputWriter;

public class ModularRawOutputWriter implements RawOutputWriter
{
   private final ArrayList<RawOutputWriter> rawOutputWriters = new ArrayList<RawOutputWriter>();
   private final String description;
   private final YoVariableRegistry registry;

   public ModularRawOutputWriter(String name, String description, RawOutputWriter rawOutputWriter)
   {
      this(name, description);
      addRawOutputWriter(rawOutputWriter);
   }

   public ModularRawOutputWriter(String name, String description, RawOutputWriter[] rawOutputWriters)
   {
      this(name, description);

      for (RawOutputWriter rawOutputWriter : rawOutputWriters)
      {
         addRawOutputWriter(rawOutputWriter);
      }
   }

   public ModularRawOutputWriter(String name, String description, List<RawOutputWriter> rawOutputWriters)
   {
      this(name, description);

      for (RawOutputWriter rawOutputWriter : rawOutputWriters)
      {
         addRawOutputWriter(rawOutputWriter);
      }
   }

   public ModularRawOutputWriter(String name, String description)
   {
      this.description = description;
      this.registry = new YoVariableRegistry(name);
   }

   public void addRawOutputWriter(RawOutputWriter rawOutputWriter)
   {
      this.rawOutputWriters.add(rawOutputWriter);
      this.registry.addChild(rawOutputWriter.getYoVariableRegistry());
   }

   @Override
   public void initialize()
   {
      for (int i = 0; i < rawOutputWriters.size(); i++)
      {
         rawOutputWriters.get(i).initialize();
      }
   }

   @Override
   public void write()
   {
      for (int i = 0; i < rawOutputWriters.size(); i++)
      {
         rawOutputWriters.get(i).write();
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
      for (RawOutputWriter rawOutputWriter : rawOutputWriters)
      {
         buf.append(rawOutputWriter.getClass().getSimpleName() + "\n");
      }
      return buf.toString();
   }
}

