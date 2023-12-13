package us.ihmc.robotics.sensors;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Objects;

import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.tools.EuclidCoreIOTools;
import us.ihmc.robotics.screwTheory.GenericCRC32;

public class ForceSensorDataHolder implements ForceSensorDataHolderReadOnly, Settable<ForceSensorDataHolder>
{
   private final List<ForceSensorDefinition> forceSensorDefinitions = new ArrayList<>();
   private final List<ForceSensorData> forceSensorDatas = new ArrayList<>();

   private final transient Map<String, ForceSensorPackage> sensorNameToDefintionMap = new LinkedHashMap<>();

   public ForceSensorDataHolder()
   {
   }

   public ForceSensorDataHolder(ForceSensorDefinition[] forceSensors)
   {
      for (ForceSensorDefinition forceSensorDefinition : forceSensors)
      {
         registerForceSensor(forceSensorDefinition);
      }
   }

   public ForceSensorDataHolder(List<ForceSensorDefinition> forceSensors)
   {
      for (ForceSensorDefinition forceSensorDefinition : forceSensors)
      {
         registerForceSensor(forceSensorDefinition);
      }
   }

   public void clear()
   {
      forceSensorDefinitions.clear();
      forceSensorDatas.clear();
   }

   public ForceSensorData registerForceSensor(ForceSensorDefinition forceSensorDefinition)
   {
      String sensorName = forceSensorDefinition.getSensorName();
      ForceSensorPackage sensorPackage = sensorNameToDefintionMap.get(sensorName);

      if (sensorPackage == null)
      {
         sensorPackage = new ForceSensorPackage(forceSensorDefinition);
         sensorNameToDefintionMap.put(sensorName, sensorPackage);
      }
      else
      {
         sensorPackage.definition.set(forceSensorDefinition);
      }

      forceSensorDefinitions.add(sensorPackage.definition);
      forceSensorDatas.add(sensorPackage.data);
      return sensorPackage.data;
   }

   @Override
   public List<ForceSensorDefinition> getForceSensorDefinitions()
   {
      return forceSensorDefinitions;
   }

   public List<ForceSensorData> getForceSensorDatas()
   {
      return forceSensorDatas;
   }

   @Override
   public ForceSensorData getData(ForceSensorDefinition sensorDefinition)
   {
      return getData(sensorDefinition.getSensorName());
   }

   @Override
   public ForceSensorData getData(String sensorName)
   {
      ForceSensorPackage sensorPackage = sensorNameToDefintionMap.get(sensorName);
      return sensorPackage == null ? null : sensorPackage.data;
   }

   @Override
   public ForceSensorDefinition getDefinition(String sensorName)
   {
      ForceSensorPackage sensorPackage = sensorNameToDefintionMap.get(sensorName);
      return sensorPackage == null ? null : sensorPackage.definition;
   }

   public int getNumberOfForceSensors()
   {
      return forceSensorDefinitions.size();
   }

   @Override
   public void set(ForceSensorDataHolder other)
   {
      clear();

      for (int i = 0; i < other.getForceSensorDefinitions().size(); i++)
      {
         ForceSensorDefinition definition = other.getForceSensorDefinitions().get(i);
         ForceSensorData data = other.getForceSensorDatas().get(i);
         registerForceSensor(definition).set(data);
      }
   }

   public void set(ForceSensorDataHolderReadOnly other)
   {
      clear();

      for (int i = 0; i < other.getForceSensorDefinitions().size(); i++)
      {
         ForceSensorDefinition forceSensorDefinition = other.getForceSensorDefinitions().get(i);
         registerForceSensor(forceSensorDefinition).set(other.getData(forceSensorDefinition));
      }
   }

   @Override
   public boolean equals(Object obj)
   {
      if (obj == this)
      {
         return true;
      }
      else if (obj instanceof ForceSensorDataHolder other)
      {
         if (forceSensorDefinitions.size() != other.forceSensorDefinitions.size())
            return false;
         for (int i = 0; i < forceSensorDefinitions.size(); i++)
         {
            if (!forceSensorDefinitions.get(i).equals(other.forceSensorDefinitions.get(i)))
               return false;
            if (!forceSensorDatas.get(i).equals(other.forceSensorDatas.get(i)))
               return false;
         }
         return true;
      }
      else
      {
         return false;
      }
   }

   public void calculateChecksum(GenericCRC32 checksum)
   {
      for (int i = 0; i < forceSensorDatas.size(); i++)
      {
         forceSensorDatas.get(i).calculateChecksum(checksum);
      }
   }

   private static class ForceSensorPackage
   {
      private final ForceSensorDefinition definition;
      private final ForceSensorData data;

      public ForceSensorPackage(ForceSensorDefinition source)
      {
         definition = new ForceSensorDefinition(source);
         data = new ForceSensorData(definition);
      }

      @Override
      public boolean equals(Object object)
      {
         if (object == this)
            return true;
         else if (object instanceof ForceSensorPackage other)
            return Objects.equals(definition, other.definition) && Objects.equals(data, other.data);
         else
            return false;
      }
   }

   @Override
   public String toString()
   {
      return "Force sensor data: "
             + EuclidCoreIOTools.getCollectionString("[\n\t",
                                                     "\n]",
                                                     "\n\t",
                                                     forceSensorDatas,
                                                     data -> "sensorName=" + data.getSensorName() + ", value=" + Arrays.toString(data.getWrenchMatrix().data));
   }
}
