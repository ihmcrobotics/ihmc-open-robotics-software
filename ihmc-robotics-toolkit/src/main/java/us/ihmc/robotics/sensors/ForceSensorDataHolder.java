package us.ihmc.robotics.sensors;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.mecano.spatial.Wrench;
import us.ihmc.mecano.spatial.interfaces.WrenchReadOnly;
import us.ihmc.robotics.screwTheory.GenericCRC32;

public class ForceSensorDataHolder implements ForceSensorDataHolderReadOnly, Settable<ForceSensorDataHolder>
{
   private final List<ForceSensorDefinition> forceSensorDefinitions = new ArrayList<>();
   private final List<ForceSensorData> forceSensorDatas = new ArrayList<>();

   private final transient Map<ForceSensorDefinition, ForceSensorData> forceSensorMap = new HashMap<>();
   private final transient Map<String, ForceSensorDefinition> sensorNameToDefintionMap = new HashMap<>();

   public ForceSensorDataHolder()
   {
   }

   public ForceSensorDataHolder(List<ForceSensorDefinition> forceSensors)
   {
      for (ForceSensorDefinition forceSensorDefinition : forceSensors)
      {
         registerForceSensor(forceSensorDefinition);
      }
   }

   public ForceSensorData registerForceSensor(ForceSensorDefinition forceSensorDefinition)
   {
      ForceSensorData forceSensorData = forceSensorMap.get(forceSensorDefinition);
      if (forceSensorData == null)
      {
         forceSensorData = new ForceSensorData();
         forceSensorMap.put(forceSensorDefinition, forceSensorData);
      }
      forceSensorDatas.add(forceSensorData);

      ForceSensorDefinition definition = sensorNameToDefintionMap.get(forceSensorDefinition.getSensorName());
      if (definition == null)
      {
         definition = new ForceSensorDefinition();
         sensorNameToDefintionMap.put(forceSensorDefinition.getSensorName(), definition);
      }
      forceSensorDefinitions.add(definition);

      forceSensorData.setFrameAndBody(forceSensorDefinition);
      definition.set(forceSensorDefinition);
      return forceSensorData;
   }

   public void registerForceSensor(ForceSensorDefinition forceSensorDefinition, ForceSensorDataReadOnly forceSensorData)
   {
      registerForceSensor(forceSensorDefinition).set(forceSensorData);
   }

   public void clear()
   {
      forceSensorDefinitions.clear();
      forceSensorDatas.clear();
   }

   @Override
   public ForceSensorData get(ForceSensorDefinition forceSensor)
   {
      return forceSensorMap.get(forceSensor);
   }

   @Override
   public List<ForceSensorDefinition> getForceSensorDefinitions()
   {
      return forceSensorDefinitions;
   }

   @Override
   public ForceSensorData getByName(String sensorName)
   {
      ForceSensorDefinition forceSensorDefinition = findForceSensorDefinition(sensorName);

      if (forceSensorDefinition == null)
         throw new RuntimeException("Force sensor not found " + sensorName);
      else
         return get(forceSensorDefinition);
   }

   @Override
   public ForceSensorDefinition findForceSensorDefinition(String sensorName)
   {
      return sensorNameToDefintionMap.get(sensorName);
   }

   public int getNumberOfForceSensors()
   {
      return forceSensorDefinitions.size();
   }

   @Override
   public void set(ForceSensorDataHolder other)
   {
      set((ForceSensorDataHolderReadOnly) other);
   }

   public void set(ForceSensorDataHolderReadOnly other)
   {
      clear();
      for (int i = 0; i < other.getForceSensorDefinitions().size(); i++)
      {
         ForceSensorDefinition forceSensorDefinition = other.getForceSensorDefinitions().get(i);
         registerForceSensor(forceSensorDefinition, other.get(forceSensorDefinition));
      }
   }

   private final transient DenseMatrix64F tempWrench = new DenseMatrix64F(6, 1);
   @Deprecated // maintains compatibility with the thread data synchronizer
   public void setDataOnly(ForceSensorDataHolder other)
   {
      for (int i = 0; i < other.getForceSensorDefinitions().size(); i++)
      {
         ForceSensorDefinition forceSensorDefinition = other.getForceSensorDefinitions().get(i);
         ForceSensorData thisData = getByName(forceSensorDefinition.getSensorName());
         ForceSensorData otherData = other.getByName(forceSensorDefinition.getSensorName());
         otherData.getWrench(tempWrench);
         thisData.setWrench(tempWrench);
      }
   }

   public void setForceSensorValue(ForceSensorDefinition key, DenseMatrix64F data)
   {
      forceSensorMap.get(key).setWrench(data);
   }

   public void setForceSensorValue(ForceSensorDefinition key, WrenchReadOnly wrench)
   {
      forceSensorMap.get(key).setWrench(wrench);
   }

   @Override
   public void getForceSensorValue(ForceSensorDefinition key, Wrench wrenchToPack)
   {
      forceSensorMap.get(key).getWrench(wrenchToPack);
   }

   @Override
   public void getForceSensorValue(ForceSensorDefinition key, DenseMatrix64F wrenchToPack)
   {
      forceSensorMap.get(key).getWrench(wrenchToPack);
   }

   @Override
   public boolean equals(Object obj)
   {
      if (obj == this)
      {
         return true;
      }
      else if (obj instanceof ForceSensorDataHolder)
      {
         ForceSensorDataHolder other = (ForceSensorDataHolder) obj;
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
      for (int i = 0; i < forceSensorDefinitions.size(); i++)
      {
         final ForceSensorDefinition forceSensorDefinition = forceSensorDefinitions.get(i);
         forceSensorMap.get(forceSensorDefinition).calculateChecksum(checksum);
      }
   }
}
